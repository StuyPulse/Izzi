package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.util.AngleVelocity;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class SwerveDriveFerryFromCurrentPosition extends Command {

    private final SwerveDrive swerve;
    private final Odometry odometry;
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final Intake intake;

    private final AngleController controller;
    private final IStream angleVelocity;
    
    private final BStream shoot;

    public SwerveDriveFerryFromCurrentPosition() {
        swerve = SwerveDrive.getInstance();
        shooter = Shooter.getInstance();
        odometry = Odometry.getInstance();
        conveyor = Conveyor.getInstance();
        intake = Intake.getInstance();

        controller = new AnglePIDController(Assist.kP, Assist.kI, Assist.kD)
            .setOutputFilter(x -> -x);

        AngleVelocity derivative = new AngleVelocity();

        angleVelocity = IStream.create(() -> derivative.get(Angle.fromRotation2d(getTargetAngle())))
            .filtered(new LowPassFilter(Assist.ANGLE_DERIV_RC))
            // make angleVelocity contribute less once distance is less than REDUCED_FF_DIST
            // so that angular velocity doesn't oscillate
            .filtered(x -> x * Math.min(1, getDistanceToTarget() / Assist.REDUCED_FF_DIST))
            .filtered(x -> -x);

        shoot = BStream.create(() -> Math.abs(controller.getError().toDegrees()) < getAngleTolerance())
            .filtered(new BDebounce.Falling(0.4));

        addRequirements(swerve, shooter, odometry, conveyor, intake);
    }

    // returns pose of close amp corner
    private Translation2d getTargetPose() {

        Translation2d pose = Robot.isBlue()
            ? new Translation2d(0, Field.WIDTH - 1.5)
            : new Translation2d(0, 1.5);
        
        return pose;

        // Translation2d offset = odometry.getRobotVelocity()
        //     .times(NOTE_TRAVEL_TIME);
        
        // return pose.minus(offset);
    }

    private Rotation2d getTargetAngle() {
        return odometry.getPose().getTranslation().minus(getTargetPose()).getAngle();
    }

    private double getDistanceToTarget() {
        return odometry.getPose().getTranslation().getDistance(getTargetPose());
    }

    private double getAngleTolerance() {
        double distance = getDistanceToTarget();
        final double SHOT_LANDING_TOLERANCE = 1.0;

        return Math.toDegrees(Math.atan(SHOT_LANDING_TOLERANCE / distance));
    }

    @Override
    public void initialize() {
        swerve.stop();
        shooter.setTargetSpeeds(Settings.Shooter.FERRY);
    }

    @Override
    public void execute() {
        double omega = angleVelocity.get() + controller.update(
            Angle.fromRotation2d(getTargetAngle()),
            Angle.fromRotation2d(odometry.getPose().getRotation()));

        SmartDashboard.putNumber("Ferry/Angle Tolerance", getAngleTolerance());

        swerve.drive(new Vector2D(new Translation2d(0, 0)), omega);
        
        if (shoot.get()) {
            intake.acquire();
            conveyor.toShooter();
        } else {
            intake.stop();
            conveyor.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        conveyor.stop();
        shooter.setTargetSpeeds(Settings.Shooter.PODIUM_SHOT);
    }
}
