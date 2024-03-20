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

public class AmperScore extends InstantCommand {

    public static Command forSeconds(double seconds) {
        return new AmperScore()
            .andThen(new WaitCommand(seconds))
            .andThen(new AmperStop());
    }

    public static Command untilDone() {
        return new AmperScore()
            .until(() -> !Amper.getInstance().hasNote());
    }

    private final Amper amper;

    public AmperScore() {
        amper = Amper.getInstance();
        addRequirements(amper);
    }

    @Override
    public void initialize() {
        amper.amp();
    }
}
