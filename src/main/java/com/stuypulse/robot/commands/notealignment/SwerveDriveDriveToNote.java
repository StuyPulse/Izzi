/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.notealignment;

import static com.stuypulse.robot.constants.Settings.Alignment.*;
import static com.stuypulse.robot.constants.Settings.NoteDetection.*;

import com.stuypulse.robot.constants.Settings.NoteDetection;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.NoteVision;
import com.stuypulse.robot.util.HolonomicController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveToNote extends Command {

    private final SwerveDrive swerve;
    private final Odometry odometry;
    private final NoteVision vision;
    private final Intake intake;

    private final VStream drive;

    private final HolonomicController controller;
    private final BStream aligned;

    public SwerveDriveDriveToNote(Gamepad driver) {
        this.swerve = SwerveDrive.getInstance();
        this.odometry = Odometry.getInstance();
        this.vision = NoteVision.getInstance();
        this.intake = Intake.getInstance();

        drive = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL.get()),
                new VLowPassFilter(Drive.RC.get()));

        controller = new HolonomicController(
            new PIDController(NoteDetection.Translation.kP, NoteDetection.Translation.kI, NoteDetection.Translation.kD),
            new PIDController(NoteDetection.Translation.kP, NoteDetection.Translation.kI, NoteDetection.Translation.kD),
            new AnglePIDController(NoteDetection.Rotation.kP, NoteDetection.Rotation.kI, NoteDetection.Rotation.kD));

        SmartDashboard.putData("Note Detection/Controller", controller);

        aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(DEBOUNCE_TIME));

        addRequirements(swerve);
    }

    private boolean isAligned() {
        return controller.isDone(THRESHOLD_X.get(), THRESHOLD_Y.get(), THRESHOLD_ANGLE.get());
    }

    @Override
    public void execute() {
        Translation2d targetTranslation = odometry.getPose()
            .getTranslation().plus(
                new Translation2d(Swerve.CENTER_TO_INTAKE_FRONT, 0)
                    .rotateBy(odometry.getPose().getRotation()));

        Rotation2d targetRotation = vision.getEstimatedNotePose().minus(targetTranslation).getAngle();

        Pose2d targetPose = new Pose2d(targetTranslation, targetRotation);

        if (vision.hasNoteData()) {
            ChassisSpeeds speeds = controller.update(targetPose, odometry.getPose());

            if (vision.withinIntakePath())
                speeds.omegaRadiansPerSecond = Math.signum(speeds.omegaRadiansPerSecond) * Math.toRadians(5.0);

            // drive to note
            swerve.setChassisSpeeds(speeds);
        } else {
            swerve.drive(drive.get(), 0);
        }

        SmartDashboard.putBoolean("Note Detection/Is Aligned", aligned.get());
    }

    @Override
    public boolean isFinished() {
        return aligned.get() || intake.hasNote();
    }
}
