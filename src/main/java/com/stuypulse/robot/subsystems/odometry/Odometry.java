/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.odometry;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.util.LinearRegression;
import com.stuypulse.robot.util.vision.AprilTag;
import com.stuypulse.robot.util.vision.VisionData;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

/** This class handles the odometry of the robot. */
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
    private final SwerveModuleOdometry[] moduleOdometries;

    private final Field2d field;
    private final FieldObject2d odometryPose2D;
    private final FieldObject2d estimatorPose2D;
    private final FieldObject2d[] moduleOdometriesPose2d;

    private final LinearRegression xyRegression;
    private final LinearRegression thetaRegression;

    private Pose2d lastGoodPose;

    private Translation2d robotVelocity;
    private Translation2d lastPose;

    protected Odometry() {
        SwerveDrive swerve = SwerveDrive.getInstance();
        odometry = new SwerveDriveOdometry(
            swerve.getKinematics(),
            swerve.getGyroAngle(),
            swerve.getModulePositions(),
            new Pose2d());
        estimator = new SwerveDrivePoseEstimator(
            swerve.getKinematics(),
            swerve.getGyroAngle(),
            swerve.getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(
                0.1,
                0.1,
                0.1),
            VecBuilder.fill(
                0.9,
                0.9,
                10.0));

        VISION_ACTIVE = new SmartBoolean("Odometry/VISION ACTIVE", true);

        String[] moduleIds = swerve.getModuleIds();
        SwerveModulePosition[] modulePositions = swerve.getModulePositions();
        Translation2d[] offsets = swerve.getModuleOffsets();
        moduleOdometries = new SwerveModuleOdometry[moduleIds.length];

        Pose2d initialPose = estimator.getEstimatedPosition();
        for (int i = 0; i < moduleIds.length; i++) {
            moduleOdometries[i] = new SwerveModuleOdometry(
                initialPose,
                modulePositions[i],
                offsets[i],
                moduleIds[i]
            );
        }

        field = new Field2d();
        swerve.initFieldObject(field);
        odometryPose2D = field.getObject("Odometry Pose");
        estimatorPose2D = field.getObject("Estimator Pose");

        moduleOdometriesPose2d = new FieldObject2d[moduleOdometries.length];
        for (int i = 0; i < moduleOdometries.length; i++) {
            moduleOdometriesPose2d[i] = field.getObject(moduleIds[i] + " Module Odometry");
        }

        xyRegression = new LinearRegression(Cameras.xyStdDevs);
        thetaRegression = new LinearRegression(Cameras.thetaStdDevs);

        lastGoodPose = new Pose2d();
        
        robotVelocity = new Translation2d();
        lastPose = new Translation2d();

        SmartDashboard.putData("Field", field);
    }

    public void setVisionEnabled(boolean enabled) {
        VISION_ACTIVE.set(enabled);
    }

    public Field2d getField() {
        return field;
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public double getDistanceToTag(AprilTag tag) {
        return getPose()
            .getTranslation()
            .getDistance(tag.getLocation().getTranslation().toTranslation2d());
    }

    public SwerveModuleOdometry[] getModuleOdometries() {
        return moduleOdometries;
    }

    /**
     * Reset the pose of the odometry to the given pose.
     *
     * @param pose the pose to reset to
     */
    public void reset(Pose2d pose) {
        SwerveDrive swerve = SwerveDrive.getInstance();
        odometry.resetPosition(swerve.getGyroAngle(), swerve.getModulePositions(), pose);
        estimator.resetPosition(swerve.getGyroAngle(), swerve.getModulePositions(), pose);
    }

    private void updateOdometry() {
        SwerveDrive swerve = SwerveDrive.getInstance();
        odometry.update(swerve.getGyroAngle(), swerve.getModulePositions());
        estimator.update(swerve.getGyroAngle(), swerve.getModulePositions());

        SwerveModulePosition[] newModulePositions = swerve.getModulePositions();
        for (int i = 0; i < moduleOdometries.length; i++) {
            moduleOdometries[i].update(newModulePositions[i]);
        }
    }

    private Vector<N3> getStandardDeviation(VisionData data) {
        double distance = data.getDistanceToPrimaryTag();

        double xyStdDev = xyRegression.calculatePoint(distance);
        double thetaStdDev = thetaRegression.calculatePoint(distance);

        SmartDashboard.putNumber("Odometry/StdDevs/XY", xyStdDev);
        SmartDashboard.putNumber("Odometry/StdDevs/Theta", thetaStdDev);

        return VecBuilder.fill(
                xyStdDev,
                xyStdDev,
                thetaStdDev);
    }

    // private void updateEstimatorWithVisionData(ArrayList<VisionData> outputs) {
    //     for (VisionData data : outputs) {
    //         estimator.addVisionMeasurement(data.getPose().toPose2d(), data.getTimestamp(), 
    //             DriverStation.isAutonomous()
    //             ? VecBuilder.fill(0.9, 0.9, 10)
    //             : VecBuilder.fill(0.7, 0.7, 10));
    //     }
    // }

    private void updateEstimatorWithVisionData(ArrayList<VisionData> outputs) {
        Pose2d poseSum = new Pose2d();
        double timestampSum = 0;
        double areaSum = 0;

        for (VisionData data : outputs) {
            Pose2d weighted = data.getPose().toPose2d().times(data.getArea());

            poseSum = new Pose2d(
                poseSum.getTranslation().plus(weighted.getTranslation()),
                poseSum.getRotation().plus(weighted.getRotation())
            );

            areaSum += data.getArea();

            timestampSum += data.getTimestamp() * data.getArea();
        }

        estimator.addVisionMeasurement(poseSum.div(areaSum), timestampSum / areaSum,
            DriverStation.isAutonomous() ? VecBuilder.fill(0.9, 0.9, 10) : VecBuilder.fill(0.7, 0.7, 10));
    }

    @Override
    public void periodic() {
        ArrayList<VisionData> outputs = AprilTagVision.getInstance().getOutputs();

        updateOdometry();

        if (VISION_ACTIVE.get() && outputs.size() > 0) {
            updateEstimatorWithVisionData(outputs);
        }

        if (estimator.getEstimatedPosition().getTranslation().getNorm() > new Translation2d(Field.LENGTH, Field.WIDTH).getNorm() || 
            odometry.getPoseMeters().getTranslation().getNorm() > new Translation2d(Field.LENGTH, Field.WIDTH).getNorm() ||
            estimator.getEstimatedPosition().getX() == Double.NaN || estimator.getEstimatedPosition().getY() == Double.NaN ||
            odometry.getPoseMeters().getX() == Double.NaN || odometry.getPoseMeters().getY() == Double.NaN
        ) {
            reset(lastGoodPose);
        } else {
            lastGoodPose = getPose();
        }

        robotVelocity = getPose().getTranslation().minus(lastPose).div(Settings.DT);
        lastPose = getPose().getTranslation();

        updateTelemetry();
    }

    public Translation2d getRobotVelocity() {
        return robotVelocity;
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Odometry/Odometry/X", odometry.getPoseMeters().getTranslation().getX());
        SmartDashboard.putNumber("Odometry/Odometry/Y", odometry.getPoseMeters().getTranslation().getY());
        SmartDashboard.putNumber("Odometry/Odometry/Rotation", odometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("Odometry/Estimator/X", estimator.getEstimatedPosition().getTranslation().getX());
        SmartDashboard.putNumber("Odometry/Estimator/Y", estimator.getEstimatedPosition().getTranslation().getY());
        SmartDashboard.putNumber("Odometry/Estimator/VX", robotVelocity.getX());
        SmartDashboard.putNumber("Odometry/Estimator/VY", robotVelocity.getY());
        SmartDashboard.putNumber("Odometry/Estimator/Rotation", estimator.getEstimatedPosition().getRotation().getDegrees());

        odometryPose2D.setPose(odometry.getPoseMeters());
        estimatorPose2D.setPose(estimator.getEstimatedPosition());
        
        for (int i = 0; i < moduleOdometriesPose2d.length; i++) {
            moduleOdometriesPose2d[i].setPose(moduleOdometries[i].getPose());
        }
    }
}
