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
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.stuylib.util.AngleVelocity;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class SwerveDriveAutoFerry extends Command {

    private final SwerveDrive swerve;
    private final Odometry odometry;
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final Intake intake;
    
    private final VStream drive;

    private final AngleController controller;
    private final IStream angleVelocity;

    public SwerveDriveAutoFerry(Gamepad driver) {
        swerve = SwerveDrive.getInstance();
        shooter = Shooter.getInstance();
        odometry = Odometry.getInstance();
        conveyor = Conveyor.getInstance();
        intake = Intake.getInstance();

        drive = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL.get()),
                new VLowPassFilter(Drive.RC.get()));

        controller = new AnglePIDController(Assist.kP, Assist.kI, Assist.kD)
            .setOutputFilter(x -> -x);

        AngleVelocity derivative = new AngleVelocity();

        angleVelocity = IStream.create(() -> derivative.get(Angle.fromRotation2d(getTargetAngle())))
            .filtered(new LowPassFilter(Assist.ANGLE_DERIV_RC))
            // make angleVelocity contribute less once distance is less than REDUCED_FF_DIST
            // so that angular velocity doesn't oscillate
            .filtered(x -> x * Math.min(1, getDistanceToTarget() / Assist.REDUCED_FF_DIST))
            .filtered(x -> -x);


        addRequirements(swerve, shooter, odometry, conveyor, intake);
    }

    // returns pose of close amp corner
    private Translation2d getTargetPose() {
        return Robot.isBlue()
            ? new Translation2d(0, Field.WIDTH - 0.75)
            : new Translation2d(0, 0.5);
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

    private boolean shouldMoveBack() {
        Translation2d robot = odometry.getPose().getTranslation();

        return robot.getX() < Field.FERRY_SHOT_MIN_X ||
            (robot.getX() < Field.FERRY_SHOT_MIN_FAR_X
            && robot.getY() < Field.WIDTH / 2.0 + 1);
    }

    private boolean canShoot() {
        Translation2d robot = odometry.getPose().getTranslation();

        if (shouldMoveBack())
            return false;
        
        if (robot.getX() > Field.FERRY_SHOT_MAX_X)
            return false;
        
        return true;
    }

    @Override
    public void initialize() {
        shooter.setTargetSpeeds(Settings.Shooter.FERRY);
    }

    @Override
    public void execute() {
        double omega = angleVelocity.get() + controller.update(
            Angle.fromRotation2d(getTargetAngle()),
            Angle.fromRotation2d(odometry.getPose().getRotation()));

        if (shouldMoveBack()) {
            swerve.setFieldRelativeSpeeds(new ChassisSpeeds(2.0, 0, -omega));

            intake.stop();
            conveyor.stop();
        } else {
            SmartDashboard.putNumber("Ferry/Angle Tolerance", getAngleTolerance());
            if (canShoot() && Math.abs(controller.getError().toDegrees()) < getAngleTolerance()) {
                intake.acquire();
                conveyor.toShooter();
            } else {
                intake.stop();
                conveyor.stop();
            }

            swerve.drive(drive.get(), omega);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        conveyor.stop();
        shooter.setTargetSpeeds(Settings.Shooter.PODIUM_SHOT);
    }
}
