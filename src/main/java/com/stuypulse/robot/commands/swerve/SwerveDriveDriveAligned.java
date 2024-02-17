package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class SwerveDriveDriveAligned extends Command {

    public static Translation2d getLookaheadOffset(double seconds) {
        SwerveDrive swerve = SwerveDrive.getInstance();

        return new Translation2d(
            swerve.getChassisSpeeds().vxMetersPerSecond,
            swerve.getChassisSpeeds().vyMetersPerSecond)
                .times(seconds);
    }

    private final SwerveDrive swerve;
    private final Odometry odometry;
    private final VStream drive;

    private final AngleController controller;

    public SwerveDriveDriveAligned(Gamepad driver) {
        odometry = Odometry.getInstance();
        swerve = SwerveDrive.getInstance();

        drive = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> Settings.vpow(x, Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL.get()),
                new VLowPassFilter(Drive.RC.get()));

        controller = new AnglePIDController(Rotation.kP, Rotation.kI, Rotation.kD)
            .setOutputFilter(x -> -x);
        
        addRequirements(swerve);
    }

    public abstract Rotation2d getTargetAngle();

    @Override
    public void execute() {
        swerve.drive(drive.get(), controller.update(
            Angle.fromRotation2d(getTargetAngle()),
            Angle.fromRotation2d(odometry.getPose().getRotation())));
    }
}
