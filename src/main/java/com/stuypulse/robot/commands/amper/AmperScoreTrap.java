/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.conveyor.ConveyorMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AmperScoreTrap extends ConveyorSetMode {

    public static Command forSeconds(double seconds) {
        return new AmperScoreTrap()
            .andThen(new WaitCommand(seconds))
            .andThen(new AmperStop());
    }

    public static Command untilDone() {
        return new AmperScoreTrap()
            .andThen(new WaitUntilCommand(() -> !Amper.getInstance().hasNote()))
            .andThen(new AmperStop());
    }

    public AmperScoreTrap() {
        super(ConveyorMode.SCORE_TRAP);
    }
}
