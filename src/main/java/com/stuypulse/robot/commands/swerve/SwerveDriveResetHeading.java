/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveResetHeading extends InstantCommand {

    public SwerveDriveResetHeading(Rotation2d heading) {
        super(() -> {
            Odometry odometry = Odometry.getInstance();
            odometry.reset(new Pose2d(odometry.getPose().getTranslation(), heading));
        });
    }

    public SwerveDriveResetHeading() {
        this(new Rotation2d());
    }
}
