package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveToPose extends Command {

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
            .filtered(new BDebounceRC.Both(Alignment.DEBOUNCE_TIME.get()));
        
        xTolerance = Alignment.X_TOLERANCE.get();
        yTolerance = Alignment.Y_TOLERANCE.get();
        thetaTolerance = Alignment.ANGLE_TOLERANCE.get();

        addRequirements(swerve);
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
        swerve.setChassisSpeeds(controller.getOutput());
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        targetPose2d.setPose(Double.NaN, Double.NaN, new Rotation2d(Double.NaN));
    }

}
