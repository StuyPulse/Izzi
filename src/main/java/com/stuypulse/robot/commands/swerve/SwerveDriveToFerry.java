/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;


import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDriveToFerry extends SwerveDriveToPose {

    public SwerveDriveToFerry() {
        super(() -> {
            Pose2d robot = Odometry.getInstance().getPose();

            Translation2d target = Robot.isBlue()
                ? new Translation2d(0, Field.WIDTH - 1.5)
                : new Translation2d(0, 1.5);
        
            Rotation2d targetAngle = new Translation2d(Field.CONST_FERRY_X, robot.getY())
                .minus(target)
                .getAngle();

            // use robot's y, put in x and rotation

            return new Pose2d(
                SLMath.clamp(robot.getX(), Field.FERRY_SHOT_MIN_X, Field.FERRY_SHOT_MAX_X),
                Robot.isBlue() ? Math.max(robot.getY(), Field.FERRY_CUTOFF) : Math.min(robot.getY(), Field.WIDTH - Field.FERRY_CUTOFF),
                targetAngle
            );
        });
    }
}