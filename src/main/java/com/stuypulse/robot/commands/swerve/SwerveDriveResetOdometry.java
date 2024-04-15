/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveResetOdometry extends InstantCommand {

    public SwerveDriveResetOdometry(Supplier<Pose2d> supplier) {
        super(() -> Odometry.getInstance().reset(supplier.get()));
    }

    public SwerveDriveResetOdometry(Pose2d pose) {
        super(() -> Odometry.getInstance().reset(pose));
    }

    public SwerveDriveResetOdometry() {
        this(new Pose2d());
    }
}
