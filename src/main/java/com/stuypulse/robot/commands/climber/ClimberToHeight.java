/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.commands.DoNothingCommand;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climber.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimberToHeight extends InstantCommand {

    public static Command untilDone(double height) {
        return new ClimberToHeight(height)
            .andThen(new DoNothingCommand().until(() -> Climber.getInstance().isAtTargetHeight(Settings.Climber.BangBang.THRESHOLD)));
    }

    private final Climber climber;
    private final double height;

    public ClimberToHeight(double height) {
        climber = Climber.getInstance();
        this.height = height;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setTargetHeight(height);
    }
}
