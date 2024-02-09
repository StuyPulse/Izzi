package com.stuypulse.robot.commands.swerve;

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

    private final HolonomicController controller;
    private final Pose2d targetPose;
    private final BStream isAligned;

    private final FieldObject2d targetPose2d;

    private double xTolerance;
    private double yTolerance;
    private double thetaTolerance;

    public SwerveDriveToPose(Pose2d targetPose) {
        swerve = SwerveDrive.getInstance();
        this.targetPose = targetPose;

        targetPose2d = Odometry.getInstance().getField().getObject("Target Pose");

        controller = new HolonomicController(
            new PIDController(Translation.P, Translation.I, Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new AnglePIDController(Rotation.P, Rotation.I, Rotation.D));

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
    
    private boolean isAligned() {
        return controller.isDone(xTolerance, yTolerance, thetaTolerance);
    }

    @Override
    public void execute() {
        Pose2d currentPose = Odometry.getInstance().getPose();
        targetPose2d.setPose(targetPose);

        controller.update(targetPose, currentPose);
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
