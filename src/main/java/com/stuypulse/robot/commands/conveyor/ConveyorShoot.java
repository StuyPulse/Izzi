/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.subsystems.conveyor.Conveyor;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ConveyorShoot extends InstantCommand {

    private final Conveyor conveyor;

    public ConveyorShoot() {
        conveyor = Conveyor.getInstance();

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.shoot();
    }
}
