/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.subsystems.climber.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimberToBottom extends InstantCommand {

    private final Climber climber;

    public ClimberToBottom() {
        climber = Climber.getInstance();
    }

    @Override
    public void initialize() {
        climber.toBottom();
    }

}
