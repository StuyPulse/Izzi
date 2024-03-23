/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.pathplanner.lib.util.PIDConstants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.util.MirrorRotation2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class SwerveDriveToPose extends Command {

    public static SwerveDriveToPose speakerRelative(double angleToSpeaker, double distanceToSpeaker) {
        MirrorRotation2d angle = MirrorRotation2d.fromBlue(
            Rotation2d.fromDegrees(SLMath.clamp(
                angleToSpeaker, Alignment.PODIUM_SHOT_MAX_ANGLE)));

        double distance = SLMath.clamp(distanceToSpeaker, 1, 5);

        return new SwerveDriveToPose(() -> {
            return new Pose2d(
                Field.getAllianceSpeakerPose().getTranslation()
                    .plus(new Translation2d(distance, angle.get())),
                angle.get());
            }
        );
    }

    public static SwerveDriveToPose speakerRelative(double angleToSpeaker) {
        return speakerRelative(angleToSpeaker, Alignment.PODIUM_SHOT_DISTANCE.get());
    }

    private final SwerveDrive swerve;
    private final Odometry odometry;

    private final HolonomicController controller;
    private final Supplier<Pose2d> poseSupplier;
    private final BStream isAligned;

    private final FieldObject2d targetPose2d;

    private double xTolerance;
    private double yTolerance;
    private double thetaTolerance;

    private Pose2d targetPose;

    public SwerveDriveToPose(Pose2d targetPose) {
        this(() -> targetPose);
    }

    public SwerveDriveToPose(Supplier<Pose2d> poseSupplier) {
        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();

        this.poseSupplier = poseSupplier;

        targetPose2d = odometry.getField().getObject("Target Pose");

        controller = new HolonomicController(
            new PIDController(Translation.kP, Translation.kI, Translation.kD),
            new PIDController(Translation.kP, Translation.kI, Translation.kD),
            new AnglePIDController(Rotation.kP, Rotation.kI, Rotation.kD));

        isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Both(Alignment.DEBOUNCE_TIME));

        xTolerance = Alignment.X_TOLERANCE.get();
        yTolerance = Alignment.Y_TOLERANCE.get();
        thetaTolerance = Alignment.ANGLE_TOLERANCE.get();

        addRequirements(swerve);
    }
    
    public SwerveDriveToPose withTranslationConstants(PIDConstants pid) {
        controller.setTranslationConstants(pid.kP, pid.kI, pid.kD);
        return this;
    }
    
    public SwerveDriveToPose withRotationConstants(PIDConstants pid) {
        controller.setRotationConstants(pid.kP, pid.kI, pid.kD);
        return this;
    }

    public SwerveDriveToPose withTranslationConstants(double p, double i, double d) {
        controller.setTranslationConstants(p, i, d);
        return this;
    }
    
    public SwerveDriveToPose withRotationConstants(double p, double i, double d) {
        controller.setRotationConstants(p, i, d);
        return this;
    }

    public SwerveDriveToPose withTolerance(Number x, Number y, Number theta) {
        xTolerance = x.doubleValue();
        yTolerance = y.doubleValue();
        thetaTolerance = theta.doubleValue();
        return this;
    }

    @Override
    public void initialize() {
        targetPose = poseSupplier.get();
    }

    private boolean isAligned() {
        return controller.isDone(xTolerance, yTolerance, thetaTolerance);
    }

    @Override
    public void execute() {
        targetPose2d.setPose(targetPose);
        controller.update(targetPose, odometry.getPose());

        Vector2D speed = new Vector2D(controller.getOutput().vxMetersPerSecond, controller.getOutput().vyMetersPerSecond)
            .clamp(Swerve.MAX_MODULE_SPEED);
        double rotation = SLMath.clamp(controller.getOutput().omegaRadiansPerSecond, Motion.MAX_ANGULAR_VELOCITY.get());
        
        SmartDashboard.putNumber("Alignment/Translation Target Speed", speed.distance());

        if (Math.abs(rotation) < Swerve.ROTATION_DEADBAND.get())
            rotation = 0;

        ChassisSpeeds clamped = new ChassisSpeeds(
            speed.x, speed.y, rotation);
        
        swerve.setChassisSpeeds(clamped);

    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        Field.clearFieldObject(targetPose2d);
    }
}
