/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveXMode extends Command {
    private final SwerveDrive swerve;

    public SwerveDriveXMode() {
        swerve = SwerveDrive.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.setXMode();
    }
}
