package com.stuypulse.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModuleOdometry{
  private Pose2d pose;
  private SwerveModulePosition previousModulePosition;

  private Translation2d offset;

  public SwerveModuleOdometry(SwerveModulePosition modulePosition, Translation2d offset, String moduleId) {
    this.previousModulePosition = modulePosition;
    this.offset = offset;

    Odometry robotOdometry = Odometry.getInstance();
    this.pose = new Pose2d(
      robotOdometry.getPose().getTranslation().plus(offset.rotateBy(robotOdometry.getPose().getRotation())),
      modulePosition.angle.plus(robotOdometry.getPose().getRotation())
    );
  }

  /**
   * Resets the modules's position to where it "should" be relative to the robot's odometry. 
   */
  public void resetPositionToRobot() {
    this.pose = new Pose2d(
      pose.getTranslation().plus(offset.rotateBy(Odometry.getInstance().getPose().getRotation())),
      previousModulePosition.angle.plus(Odometry.getInstance().getPose().getRotation())
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
   *
   * @return The new pose of the module.
   */
  public Pose2d update(SwerveModulePosition newPosition) {
    Rotation2d angle = newPosition.angle;

    double dx = angle.getCos() * (newPosition.distanceMeters - previousModulePosition.distanceMeters);
    double dy = angle.getSin() * (newPosition.distanceMeters - previousModulePosition.distanceMeters);

    previousModulePosition = newPosition.copy();
    pose = new Pose2d(pose.getX() + dx, pose.getY() + dy, angle);

    return pose;
  }
}
