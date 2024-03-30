/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Alignment.Shoot;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDerivative;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveToShootMoving extends Command {

    public static SwerveDriveToShootMoving withHigherDebounce() {
        return new SwerveDriveToShootMoving(Alignment.PODIUM_SHOT_DISTANCE, 0.75);
    }

    private final SwerveDrive swerve;
    private final Odometry odometry;
    private final Conveyor conveyor;
    private final Intake intake;

    private final PIDController distanceController;
    private final AnglePIDController angleController;

    private final IStream distanceToSpeaker;
    private final BStream isAligned;

    private final VStream robotPose;
    private final VStream robotVelocity;
    private final VStream projectedRobotPose;

    private final Number targetDistance;

    private double distanceTolerance;
    private double angleTolerance;

    public SwerveDriveToShootMoving() {
        this(Alignment.PODIUM_SHOT_DISTANCE);
    }

    public SwerveDriveToShootMoving(Number targetDistance) {
        this(targetDistance, Alignment.DEBOUNCE_TIME);
    }

    public SwerveDriveToShootMoving(Number targetDistance, double debounce) {
        this.targetDistance = targetDistance;

        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();
        conveyor = Conveyor.getInstance();
        intake = Intake.getInstance();

        distanceController = new PIDController(Shoot.Translation.kP, Shoot.Translation.kI, Shoot.Translation.kD);
        
        angleController = new AnglePIDController(Shoot.Rotation.kP, Shoot.Rotation.kI, Shoot.Rotation.kD);

        robotPose = VStream.create(() -> new Vector2D(odometry.getPose().getTranslation()));

        robotVelocity = robotPose
            .filtered(new VDerivative())
            .filtered(new VLowPassFilter(Alignment.PROJECTED_POSE_RC))
            .polling(0.02);

        projectedRobotPose = robotVelocity
            .filtered(v -> v.mul(Alignment.NOTE_TO_GOAL_TIME))
            .add(robotPose);

        distanceToSpeaker = IStream.create(() -> Field.getAllianceSpeakerPose().getTranslation().minus(projectedRobotPose.get().getTranslation2d()).getNorm())
            .filtered(new LowPassFilter(0.05));

        isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Rising(debounce));
        
        distanceTolerance = 0.033;
        angleTolerance = Alignment.ANGLE_TOLERANCE.get();

        addRequirements(swerve);
    }

    private double getTargetDistance() {
        return SLMath.clamp(targetDistance.doubleValue(), 1, 5);
    }

    public SwerveDriveToShootMoving withTolerance(double distanceTolerance, double angleTolerance) {
        this.distanceTolerance = distanceTolerance;
        this.angleTolerance = angleTolerance;
        return this;
    }
    
    public SwerveDriveToShootMoving withDistanceConstants(double p, double i, double d) {
        distanceController.setPID(p, i, d);
        return this;
    }
    
    public SwerveDriveToShootMoving withRotationConstants(double p, double i, double d) {
        angleController.setPID(p, i, d);
        return this;
    }

    private boolean isAligned() {
        return distanceController.isDone(distanceTolerance)
            && angleController.isDoneDegrees(angleTolerance);
    }

    @Override
    public void execute() {
        Rotation2d toSpeaker = Field.getAllianceSpeakerPose().getTranslation()
            .minus(projectedRobotPose.get().getTranslation2d())
            .getAngle();
        
        double speed = -distanceController.update(getTargetDistance(), distanceToSpeaker.get());
        double rotation = angleController.update(
            Angle.fromRotation2d(toSpeaker).add(Angle.k180deg),
            Angle.fromRotation2d(odometry.getPose().getRotation()));

        Translation2d speeds = new Translation2d(
            speed,
            toSpeaker);

        if (Math.abs(rotation) < Swerve.ALIGN_OMEGA_DEADBAND.get())
            rotation = 0;

        swerve.setFieldRelativeSpeeds(
            new ChassisSpeeds(
                speeds.getX(),
                speeds.getY(),
                rotation));
        
        if (isAligned.get()) {
            conveyor.toShooter();
            intake.acquire();
        } else {
            conveyor.stop();
            intake.stop();
        }

        SmartDashboard.putNumber("Alignment/To Shoot Target Angle", toSpeaker.plus(Rotation2d.fromDegrees(180)).getDegrees());
        SmartDashboard.putNumber("Alignment/Velocity X", robotVelocity.get().x);
        SmartDashboard.putNumber("Alignment/Velocity Y", robotVelocity.get().y);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        intake.stop();
        conveyor.stop();
    }
}