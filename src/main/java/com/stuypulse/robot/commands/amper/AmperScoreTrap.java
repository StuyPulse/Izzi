/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AmperScoreTrap extends InstantCommand {

    public static Command forSeconds(double seconds) {
        return new AmperScoreTrap()
            .andThen(new WaitCommand(seconds))
            .andThen(new AmperStop());
    }

    public static Command untilDone() {
        return new AmperScoreTrap()
            .until(() -> !Amper.getInstance().hasNote());
    }

    private final Amper amper;

    public AmperScoreTrap() {
        amper = Amper.getInstance();
        addRequirements(amper);
    }

    @Override
    public void initialize() {
        amper.trap();
    }
}
