package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.robot.subsystems.odometry.Odometry;
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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class SwerveDriveDriveAligned extends Command {

    private final SwerveDrive swerve;
    private final Odometry odometry;
    private final VStream drive;

    private final AngleController controller;
    private final IStream angleVelocity;

    public SwerveDriveDriveAligned(Gamepad driver) {
        odometry = Odometry.getInstance();
        swerve = SwerveDrive.getInstance();

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
        
        addRequirements(swerve);
    }

    public abstract Rotation2d getTargetAngle();

    public abstract double getDistanceToTarget();

    @Override
    public void execute() {
        swerve.drive(
            drive.get(),
            angleVelocity.get() + controller.update(
                Angle.fromRotation2d(getTargetAngle()),
                Angle.fromRotation2d(odometry.getPose().getRotation())));
    }
}
