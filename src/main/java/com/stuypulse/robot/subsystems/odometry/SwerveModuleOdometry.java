package com.stuypulse.robot.subsystems.odometry;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModuleOdometry {
    private Pose2d pose;
    private SwerveModulePosition previousModulePosition;
    private Translation2d offset;
    private StopWatch stopwatch;

    public SwerveModuleOdometry(Pose2d initialRobotPose, SwerveModulePosition modulePosition, Translation2d offset) {
        this.previousModulePosition = modulePosition;
        this.offset = offset;
        this.pose = new Pose2d(
                initialRobotPose.getTranslation().plus(offset.rotateBy(initialRobotPose.getRotation())),
                modulePosition.angle.plus(initialRobotPose.getRotation()));
        this.stopwatch = new StopWatch();
    }

    /**
     * Resets the modules's position to where it "should" be relative to the robot's
     * odometry.
     */
    public void resetPositionToRobot() {
        Pose2d robotPose = Odometry.getInstance().getPose();
        this.pose = new Pose2d(
            robotPose.getTranslation().plus(offset.rotateBy(robotPose.getRotation())),
            pose.getRotation()
        );
    }

    /**
     * @return The pose of the module.
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Updates the modules position on the field.
     * @return The new pose of the module.
     */
    public Pose2d update(SwerveModulePosition newPosition) {
        if (stopwatch.getTime() > Settings.Swerve.SKID_TIME_BETWEEN_RESETS) {
            resetPositionToRobot();
            stopwatch.reset();
        }
        else {
            Rotation2d robotAngle = Odometry.getInstance().getPose().getRotation();
            Rotation2d moduleAngle = newPosition.angle.plus(robotAngle);

            double dx = moduleAngle.getCos() * (newPosition.distanceMeters - previousModulePosition.distanceMeters);
            double dy = moduleAngle.getSin() * (newPosition.distanceMeters - previousModulePosition.distanceMeters);

            previousModulePosition = newPosition;
            pose = new Pose2d(pose.getX() + dx, pose.getY() + dy, moduleAngle);
        }

        return pose;
    }
}
