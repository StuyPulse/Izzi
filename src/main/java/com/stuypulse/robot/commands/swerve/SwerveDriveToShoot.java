package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDriveToShoot extends SwerveDriveToPose {

    private static Pose2d getSpeakerTargetPose() {
        Translation2d speaker = Field.getAllianceSpeakerPose().getTranslation();
        Translation2d robot = Odometry.getInstance().getPose().getTranslation();

        Translation2d speakerToRobot = robot.minus(speaker);
        
        // limits maximum angle to speaker
        // https://www.desmos.com/calculator/vj4sn6mv6m
        double heightLimit = Math.abs(speakerToRobot.getX() * Math.tan(Math.toRadians(Alignment.PODIUM_SHOT_MAX_ANGLE.get())));
        speakerToRobot = new Translation2d(speakerToRobot.getX(), MathUtil.clamp(speakerToRobot.getY(), -heightLimit, heightLimit));
       
        speakerToRobot = speakerToRobot.div(speakerToRobot.getNorm());

        return new Pose2d(
            speaker.plus(speakerToRobot.times(Alignment.PODIUM_SHOT_DISTANCE.get())),
            speakerToRobot.getAngle().plus(Rotation2d.fromDegrees(180)));
    }

    public SwerveDriveToShoot() {
        super(() -> getSpeakerTargetPose());
    }
}
