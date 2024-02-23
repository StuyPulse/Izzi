/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ConveyorShoot extends InstantCommand {

    private final Conveyor conveyor;
    private final Intake intake;

    public ConveyorShoot() {
        conveyor = Conveyor.getInstance();
        intake = Intake.getInstance();

        addRequirements(conveyor, intake);
    }

    @Override
    public void initialize() {
        conveyor.toShooter();
        intake.acquire();
    }
}
