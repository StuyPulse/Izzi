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

public class AmperScoreSpeed extends InstantCommand {

    public static Command forSeconds(double speed, double seconds) {
        return new AmperScoreSpeed(speed)
            .andThen(new WaitCommand(seconds))
            .andThen(new AmperStop());
    }

    public static Command untilDone(double speed) {
        return new AmperScoreSpeed(speed)
            .until(() -> !Amper.getInstance().hasNote());
    }

    private final Amper amper;

    private final double speed;

    public AmperScoreSpeed(double speed) {
        amper = Amper.getInstance();
        this.speed = speed;

        addRequirements(amper);
    }

    @Override
    public void initialize() {
        amper.run(speed);
    }
}
