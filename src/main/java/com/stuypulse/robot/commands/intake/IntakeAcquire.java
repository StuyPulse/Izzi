/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAcquire extends Command {

    private final Intake intake;
    private final Conveyor conveyor;

    public IntakeAcquire() {
        intake = Intake.getInstance();
        conveyor = Conveyor.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.acquire();
        conveyor.toShooter();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        return conveyor.isNoteAtShooter();
    }
}
