package com.stuypulse.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModuleOdometry {
    private Pose2d pose;
    private SwerveModulePosition previousModulePosition;

    private Translation2d offset;

    public SwerveModuleOdometry(Pose2d initialRobotPose, SwerveModulePosition modulePosition, Translation2d offset,
            String moduleId) {
        this.previousModulePosition = modulePosition;
        this.offset = offset;

        this.pose = new Pose2d(
                initialRobotPose.getTranslation().plus(offset.rotateBy(initialRobotPose.getRotation())),
                modulePosition.angle.plus(initialRobotPose.getRotation()));
    }

    /**
     * Resets the modules's position to where it "should" be relative to the robot's
     * odometry.
     */
    public void resetPositionToRobot() {
        this.pose = new Pose2d(
                pose.getTranslation().plus(offset.rotateBy(Odometry.getInstance().getPose().getRotation())),
                previousModulePosition.angle.plus(Odometry.getInstance().getPose().getRotation()));
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
        Rotation2d robotAngle = Odometry.getInstance().getPose().getRotation();

        Rotation2d moduleAngle = newPosition.angle.plus(robotAngle);

        double dx = moduleAngle.getCos() * (newPosition.distanceMeters - previousModulePosition.distanceMeters);
        double dy = moduleAngle.getSin() * (newPosition.distanceMeters - previousModulePosition.distanceMeters);

        previousModulePosition = newPosition;
        pose = new Pose2d(pose.getX() + dx, pose.getY() + dy, moduleAngle);

        return pose;
    }
}
