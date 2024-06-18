/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.notealignment;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.AStream;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.stuylib.util.AngleVelocity;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.NoteVision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveNoteAlignedDrive extends Command {

    private final NoteVision noteVision;
    private final SwerveDrive swerve;
    private final Odometry odometry;
    private final VStream drive;
    private final Intake intake;

    private final AngleController controller;
    private final IStream angleVelocity;

    private final AStream targetAngle;

    public SwerveDriveNoteAlignedDrive(Gamepad driver) {
		noteVision = NoteVision.getInstance();
		odometry = Odometry.getInstance();
        swerve = SwerveDrive.getInstance();
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
            .filtered(x -> x * Math.min(1, noteVision.getRobotRelativeNotePose().getDistance(new Translation2d()) / Assist.REDUCED_FF_DIST))
            .filtered(x -> -x);
        
        targetAngle = AStream.create(() -> Angle.fromRotation2d(getTargetAngle()))
            .filtered(new ARateLimit(1.0));
        
        addRequirements(swerve);
    }

    private Rotation2d getTargetAngle() {
        return noteVision.getRobotRelativeNotePose().getAngle();
    }

    @Override
    public boolean isFinished() {
        return intake.hasNote();
    }

    @Override
    public void execute() {
        double omega = angleVelocity.get() + controller.update(
            targetAngle.get(),
            Angle.fromRotation2d(odometry.getPose().getRotation()));

        if (!NoteVision.getInstance().hasNoteData())
            omega = 0;

        swerve.drive(drive.get(), omega);
    }
}
