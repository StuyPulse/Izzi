/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveDriveAmpAlign extends SequentialCommandGroup {

    // private static final double AMP_WALL_SETUP_X_TOLERANCE = Units.inchesToMeters(2.0);
    // private static final double AMP_WALL_SETUP_Y_TOLERANCE = Units.inchesToMeters(4.0);
    // private static final double AMP_WALL_SETUP_ANGLE_TOLERANCE = 5;

    private static final double AMP_WALL_SCORE_X_TOLERANCE = Units.inchesToMeters(0.75);
    private static final double AMP_WALL_SCORE_Y_TOLERANCE = Units.inchesToMeters(1.0);
    private static final double AMP_WALL_SCORE_ANGLE_TOLERANCE = 2;

    private static Pose2d getTargetPose(double distanceToWall) {
        Translation2d amp = Field.getAllianceAmpPose().getTranslation();
        Translation2d delta = new Translation2d(0, Robot.isBlue() ? -distanceToWall : distanceToWall);
        Rotation2d targetAngle = Rotation2d.fromDegrees(Robot.isBlue() ? 270 : 90);

        return new Pose2d(amp.plus(delta), targetAngle);
    }

    public SwerveDriveAmpAlign() {
        addCommands(
            new SwerveDrivePathFindTo(getTargetPose(Alignment.AMP_WALL_SETUP_DISTANCE.get())).get(),

            // new SwerveDriveToPose(() -> getTargetPose(Alignment.AMP_WALL_SETUP_DISTANCE.get()))
            //     .withTolerance(AMP_WALL_SETUP_X_TOLERANCE, AMP_WALL_SETUP_Y_TOLERANCE, AMP_WALL_SETUP_ANGLE_TOLERANCE),

            new SwerveDriveToPose(() -> getTargetPose(Alignment.AMP_WALL_SCORE_DISTANCE.get()))
                .withTolerance(
                    AMP_WALL_SCORE_X_TOLERANCE,
                    AMP_WALL_SCORE_Y_TOLERANCE,
                    AMP_WALL_SCORE_ANGLE_TOLERANCE)
        );
    }
}
