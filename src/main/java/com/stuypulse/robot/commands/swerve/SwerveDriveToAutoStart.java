package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import static com.stuypulse.robot.constants.Settings.Alignment.*;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class SwerveDriveToAutoStart extends Command {
    private final SwerveDrive swerve;
    private Pose2d targetPose;

    // Holonomic control
    private final HolonomicController controller;
    private final BStream aligned;

    private final FieldObject2d targetPose2d;

    private final RobotContainer robot;

    public SwerveDriveToAutoStart(RobotContainer robot) {
        this.swerve = SwerveDrive.getInstance();
        this.robot = robot;

        controller = new HolonomicController(
            new PIDController(Translation.kP,Translation.kI,Translation.kD),
            new PIDController(Translation.kP, Translation.kI, Translation.kD),
            new AnglePIDController(Rotation.kP, Rotation.kI, Rotation.kD));

        SmartDashboard.putData("Alignment/Controller", controller);

        aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(DEBOUNCE_TIME));

        targetPose2d = Odometry.getInstance().getField().getObject("Target Pose");
        addRequirements(swerve);
    }

    private boolean isAligned() {
        return controller.isDone(X_TOLERANCE.get(), Y_TOLERANCE.get(), ANGLE_TOLERANCE.get());
    }

    @Override
    public void initialize() {
        try {
            targetPose = PathPlannerAuto.getStaringPoseFromAutoFile(
                robot.getAutonomousCommand().getName());
        } catch (Exception e) {
            targetPose = Odometry.getInstance().getPose();
        }
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
        return aligned.get();
    }

    public void end(boolean interupted) {
        swerve.stop();
        targetPose2d.setPose(Double.NaN, Double.NaN, new Rotation2d(Double.NaN));
    }

}