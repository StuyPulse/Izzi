/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.commands.amper.AmperToHeight;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeToAmp extends Command {
    public static Command withCheckLift() {
        return AmperToHeight.untilDone(Settings.Amper.Lift.HANDOFF_HEIGHT)
            .andThen(new IntakeToAmp());
    }

    private final Shooter shooter;
    private final Intake intake;
    private final Amper amper;

    public IntakeToAmp() {
        shooter = Shooter.getInstance();
        intake = Intake.getInstance();
        amper = Amper.getInstance();

        addRequirements(intake, shooter, amper);
    }

    @Override
    public void initialize() {
        shooter.setTargetSpeeds(Settings.Shooter.HANDOFF);
    }

    @Override
    public void execute() {
        if (shooter.atTargetSpeeds()) {
            intake.acquire();
            amper.fromConveyor();
        }
    }

    @Override
    public boolean isFinished() {
        return amper.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        intake.stop();
        amper.stopRoller();
    }
}
