/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.commands.amper.AmperToHeight;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.util.ShooterSpeeds;

import edu.wpi.first.wpilibj2.command.Command;

public class ConveyorToAmp extends Command {
    public static Command withCheckLift() {
        return AmperToHeight.untilDone(Settings.Amper.Lift.MIN_HEIGHT)
            .andThen(new ConveyorToAmp());
    }

    private final Conveyor conveyor;
    private final Shooter shooter;
    private final Intake intake;
    private final Amper amper;

    public ConveyorToAmp() {
        conveyor = Conveyor.getInstance();
        shooter = Shooter.getInstance();
        intake = Intake.getInstance();
        amper = Amper.getInstance();

        addRequirements(conveyor, intake, amper);
    }

    @Override
    public void execute() {
        shooter.setTargetSpeeds(Settings.Shooter.HANDOFF);

        if (shooter.atTargetSpeeds()) {
            conveyor.toAmp();
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
        conveyor.stop();
        // shooter.stop();
        intake.stop();
        amper.stopRoller();
    }
}
