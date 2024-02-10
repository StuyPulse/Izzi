package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.*;
import com.stuypulse.robot.constants.Settings.Driver;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.NoteVision;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveAutomatic extends Command {
    
    private final SwerveDrive swerve;
    private final Intake intake;
    private final Conveyor conveyor;
    private final Amper amper;
    private final Odometry odometry;
    
    private final VStream drive;
    private final AngleController controller;
    private final NoteVision llNoteVision;
    private final Gamepad driver;

    private boolean endButtonWasFalse;

    public SwerveDriveAutomatic(Gamepad driver) {
        this.driver = driver;

        odometry = Odometry.getInstance();
        swerve = SwerveDrive.getInstance();
        intake = Intake.getInstance();
        conveyor = Conveyor.getInstance();
        amper = Amper.getInstance();
        llNoteVision = NoteVision.getInstance();

        drive = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> Settings.vpow(x, Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL.get()),
                new VLowPassFilter(Drive.RC.get())
        );
        controller = new AnglePIDController(Rotation.kP, Rotation.kI, Rotation.kD)
            .setOutputFilter(x -> -x);

        endButtonWasFalse = false;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        endButtonWasFalse = false;
    }

    @Override
    public void execute() {
        if (!endButtonWasFalse && !driver.getRawTopButton()) endButtonWasFalse = true;

        Translation2d currentPose = odometry.getPose().getTranslation();
        Translation2d targetPose = getTargetPose();

        Rotation2d currentAngle = odometry.getPose().getRotation();
        Rotation2d targetAngle;

        Translation2d speakerPose = Field.getAllianceSpeakerPose().getTranslation();
        double distanceToSpeaker = speakerPose.getDistance(currentPose);

        // if note in amp face wall 
        if (amper.hasNote()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) { 
                targetAngle = Rotation2d.fromDegrees(270.0);
            } else {
                targetAngle = Rotation2d.fromDegrees(90.0);
            }
        } else {
            Translation2d difference = targetPose.minus(currentPose);
            targetAngle = difference.getAngle();
        }

        //if in speaker switch sides of robot 
        if ((intake.hasNote() || conveyor.isNoteAtShooter()) && 
            (distanceToSpeaker < Assist.ALIGN_MIN_SPEAKER_DIST.getAsDouble())) {
                targetAngle = targetAngle.plus(Rotation2d.fromDegrees(180));
        }

        swerve.drive(drive.get(), controller.update(
            Angle.fromRotation2d(targetAngle),
            Angle.fromRotation2d(currentAngle)));

        SmartDashboard.putNumber("Swerve/Assist/Target angle", targetAngle.getDegrees());
    }

    private Translation2d getTargetPose() {
        Translation2d targetPose;

        Translation2d currentPose = odometry.getPose().getTranslation();
        Translation2d speakerPose = Field.getAllianceSpeakerPose().getTranslation();
        double distanceToSpeaker = speakerPose.getDistance(currentPose);

        //if already have note  
        if ((intake.hasNote() || conveyor.isNoteAtShooter()) && 
            (distanceToSpeaker<Assist.ALIGN_MIN_SPEAKER_DIST.getAsDouble())) {
            targetPose = speakerPose;
        } else { // if (llNoteVision.hasNoteData()) {
            targetPose = llNoteVision.getEstimatedNotePose();
        }
        return targetPose;
        
    }

    @Override
    public boolean isFinished() {
        return Math.abs(driver.getRightX()) > Driver.Turn.DEADBAND.getAsDouble() || (endButtonWasFalse && driver.getRawTopButton());
    }
}