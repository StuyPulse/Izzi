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
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.NoteVision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveNoteAlignedDrive extends Command {

    private final SwerveDrive swerve;
    private final Odometry odometry;
    private final NoteVision noteVision;

    private final VStream speed;

    private final AngleController alignController;

    public SwerveDriveNoteAlignedDrive(Gamepad driver) {
	swerve = SwerveDrive.getInstance();
	odometry = Odometry.getInstance();
	noteVision = NoteVision.getInstance();

	speed = VStream.create(driver::getLeftStick)
		.filtered(
			new VDeadZone(Drive.DEADBAND),
			x -> x.clamp(1.0),
			x -> Settings.vpow(x, Drive.POWER.get()),
			x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
			new VRateLimit(Drive.MAX_TELEOP_ACCEL),
			new VLowPassFilter(Drive.RC));

	alignController = new AnglePIDController(Rotation.kP, Rotation.kI, Rotation.kD);

	addRequirements(swerve);
    }

    @Override
    public void execute() {
	Translation2d targetTranslation = odometry.getPose()
		.getTranslation().plus(
			new Translation2d(Swerve.CENTER_TO_INTAKE_FRONT, 0)
				.rotateBy(odometry.getPose().getRotation()));

	Rotation2d targetRotation = noteVision.getEstimatedNotePose().minus(targetTranslation).getAngle();

	double angularVel = -alignController.update(
		Angle.fromRotation2d(targetRotation),
		Angle.fromRotation2d(odometry.getPose().getRotation()));

	// robot relative
	swerve.drive(speed.get(), angularVel);

	SmartDashboard.putNumber("Note Detection/Angle Output", alignController.getOutput());
    }
}
