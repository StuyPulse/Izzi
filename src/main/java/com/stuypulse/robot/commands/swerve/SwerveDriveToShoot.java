/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDriveToShoot extends SwerveDriveToPose {

    private static Pose2d getSpeakerTargetPose() {
        Translation2d speaker = Field.getAllianceSpeakerPose().getTranslation();
        Translation2d robot = Odometry.getInstance().getPose().getTranslation();

        Translation2d speakerToRobot = robot.minus(speaker);

        // limits maximum angle to speaker
        // https://www.desmos.com/calculator/vj4sn6mv6m
        double heightLimit = Math.abs(
            speakerToRobot.getX()
                * Math.tan(Math.toRadians(Alignment.PODIUM_SHOT_MAX_ANGLE)));
        
        speakerToRobot =
            new Translation2d(
                speakerToRobot.getX(),
                MathUtil.clamp(speakerToRobot.getY(), -heightLimit, heightLimit));

        speakerToRobot = speakerToRobot.div(speakerToRobot.getNorm());

        return new Pose2d(
                speaker.plus(speakerToRobot.times(Alignment.PODIUM_SHOT_DISTANCE.get())),
                speakerToRobot.getAngle());
    }

    public SwerveDriveToShoot() {
        super(() -> getSpeakerTargetPose());

        withTolerance(0.025, 0.025, 3);
    }
}
