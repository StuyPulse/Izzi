/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeToAmp extends Command {

    private final Conveyor conveyor;
    private final Intake intake;
    private final Amper amper;

    public IntakeToAmp() {
        conveyor = Conveyor.getInstance();
        intake = Intake.getInstance();
        amper = Amper.getInstance();

        addRequirements(conveyor, intake, amper);
    }

    @Override
    public void execute() {
        conveyor.toAmp();
        intake.acquire();
        amper.fromConveyor();
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        intake.stop();
        amper.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return conveyor.isNoteAtShooter();
    }
}
