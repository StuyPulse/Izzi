/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeToShooter extends Command {

    private final Conveyor conveyor;
    private final Intake intake;

    public IntakeToShooter() {
        conveyor = Conveyor.getInstance();
        intake = Intake.getInstance();

        addRequirements(conveyor, intake);
    }

    @Override
    public void execute() {
        conveyor.toShooter();
        intake.acquire();
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return conveyor.isNoteAtShooter();
    }
}
