/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.subsystems.conveyor.ConveyorMode;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ConveyorShoot extends ConveyorSetMode {

    public static Command untilDone() {
        return new ConveyorShoot()
            .andThen(new WaitUntilCommand(Shooter.getInstance()::noteShot));
    }

    public ConveyorShoot() {
        super(ConveyorMode.SHOOT);
    }

}
