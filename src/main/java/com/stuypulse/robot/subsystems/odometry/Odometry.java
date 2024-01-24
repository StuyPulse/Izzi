package com.stuypulse.robot.subsystems.odometry;

import java.util.ArrayList;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.util.VisionData;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase {
    
    private static final Odometry instance;

    static {
        instance = new Odometry();
    }

    public static Odometry getInstance() {
        return instance;
    }

    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator estimator;
    private final SmartBoolean VISION_ACTIVE;
    
    private final Field2d field;
    private final FieldObject2d odometryPose2D;
    private final FieldObject2d estimatorPose2D;

    public Odometry() {
        SwerveDrive swerve = SwerveDrive.getInstance();
        odometry = new SwerveDriveOdometry(swerve.getKinematics(), swerve.getGyroAngle(), swerve.getModulePositions(), new Pose2d());
        estimator = new SwerveDrivePoseEstimator(swerve.getKinematics(), swerve.getGyroAngle(), swerve.getModulePositions(), new Pose2d());

        VISION_ACTIVE = new SmartBoolean("Odometry/VISION ACTIVE", true);
        
        field = new Field2d();
        swerve.initFieldObject(field);
        odometryPose2D = field.getObject("Odometry Pose");
        estimatorPose2D = field.getObject("Estimator Pose");

        SmartDashboard.putData("Field", field);
   }

    public Field2d getField() {
        return field;
    }

    public void updateOdometry() {
        SwerveDrive swerve = SwerveDrive.getInstance();
        odometry.update(swerve.getGyroAngle(), swerve.getModulePositions());
        estimator.update(swerve.getGyroAngle(), swerve.getModulePositions());
    }
    
    public void updateWithVisionData(VisionData data) {
            estimator.addVisionMeasurement(data.getPose().toPose2d(), data.getTimestamp());
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public void reset(Pose2d pose) {
        SwerveDrive swerve = SwerveDrive.getInstance();
        odometry.resetPosition(swerve.getGyroAngle(), swerve.getModulePositions(), pose);
        estimator.resetPosition(swerve.getGyroAngle(), swerve.getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        ArrayList<VisionData> outputs = AprilTagVision.getInstance().getOutputs();
        
        if (VISION_ACTIVE.get()) {
            for (VisionData data : outputs) {
                updateWithVisionData(data);
            }
        }

        updateTelemetry();
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Odometry/Odometry/X", odometry.getPoseMeters().getTranslation().getX());
        SmartDashboard.putNumber("Odometry/Odometry/Y", odometry.getPoseMeters().getTranslation().getY());
        SmartDashboard.putNumber("Odometry/Odometry/Rotation", odometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("Odometry/Estimator/X", estimator.getEstimatedPosition().getTranslation().getX());
        SmartDashboard.putNumber("Odometry/Estimator/Y", estimator.getEstimatedPosition().getTranslation().getY());
        SmartDashboard.putNumber("Odometry/Estimator/Rotation", estimator.getEstimatedPosition().getRotation().getDegrees());

        odometryPose2D.setPose(odometry.getPoseMeters());
        estimatorPose2D.setPose(estimator.getEstimatedPosition());
    }
}
