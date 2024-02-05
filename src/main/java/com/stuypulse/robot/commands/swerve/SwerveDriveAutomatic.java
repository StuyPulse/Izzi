package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.*;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.amper.Amper;
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
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveAutomatic extends Command {
    
    private final SwerveDrive swerve;
    private final VStream drive;
    private final AngleController controller;
    private final Conveyor conveyor;
    private final NoteVision llNoteVision;
    private final Gamepad driver;
    private final Intake intake;
    private final Amper amper; 
    
    public SwerveDriveAutomatic(Gamepad driver) {
        this.driver = driver;
        swerve = SwerveDrive.getInstance();
        drive = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> Settings.vpow(x, Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL.get()),
                new VLowPassFilter(Drive.RC.get())
        );
        controller = new AnglePIDController(Assist.kP,Assist.kI,Assist.kD);
        conveyor = Conveyor.getInstance();
        llNoteVision = NoteVision.getInstance();
        intake = Intake.getInstance();
        amper = Amper.getInstance();

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Translation2d currentPose = Odometry.getInstance().getPose().getTranslation();
        Rotation2d currentAngle = currentPose.getAngle();
        Rotation2d targetAngle;
        Translation2d targetPose = getTargetPose();

        // if note in amp face wall 
        if (amper.hasNote()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) { 
                targetAngle = Rotation2d.fromDegrees(180.0);
            } else {
                targetAngle = new Rotation2d(0);
            }
        } else {
            Translation2d difference = targetPose.minus(currentPose);
            targetAngle = difference.getAngle();
        }

        controller.update(Angle.fromDegrees(targetAngle.getDegrees()), Angle.fromDegrees(currentAngle.getDegrees()));
        swerve.drive(drive.get(), controller.getOutput());
    }

    public Translation2d getTargetPose() {
        Translation2d targetPose;

        Translation2d currentPose = Odometry.getInstance().getPose().getTranslation();
        Translation2d speakerPose = Field.getAllianceSpeakerPose().getTranslation();
        double distanceToSpeaker = speakerPose.getDistance(currentPose);

        //if already have note  
        if ((intake.hasNote() || conveyor.isNoteAtShooter()) && 
            (distanceToSpeaker<Assist.minDistToSPeaker.getAsDouble())) {
            targetPose = speakerPose;
        } else { // if (llNoteVision.hasNoteData()) {
            targetPose = llNoteVision.getEstimatedNotePose();
        }
        return targetPose;

    }

    @Override
    public boolean isFinished() {
        return Math.abs(driver.getLeftX()) > Assist.deadband.getAsDouble() || driver.getRawStartButton();
    }

}