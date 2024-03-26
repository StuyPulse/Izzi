package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings.NoteDetection;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.NoteVision;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveToNote extends Command {

    private final NoteVision noteVision;
    private final SwerveDrive swerve;
    private final Odometry odometry;

    private final Controller distanceController;
    private final AngleController angleController;

    public SwerveDriveToNote() {
        noteVision = NoteVision.getInstance();
        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();

        distanceController = new PIDController(Translation.kP, Translation.kI, Translation.kD);
        angleController = new AnglePIDController(Rotation.kP, Rotation.kI, Rotation.kD)
            .setOutputFilter(x -> {
                if (x < 1.25) {
                    return x / 2.0;
                }

                if (x < 0.9) {
                    return 0;
                }

                return x;
            });
        
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (!noteVision.hasNoteData()) {
            swerve.setChassisSpeeds(new ChassisSpeeds(
                NoteDetection.DRIVE_SPEED.get(),
                0,
                0));

            return;
        }

        Translation2d notePose = noteVision.getEstimatedNotePose();

        Translation2d toNote = notePose.minus(odometry.getPose().getTranslation());
        
        double speed = -distanceController.update(0, toNote.getNorm());
        double rotation = angleController.update(
            Angle.fromRotation2d(toNote.getAngle()),
            Angle.fromRotation2d(odometry.getPose().getRotation()));

        Translation2d speeds = new Translation2d(
            speed,
            toNote.getAngle());

        swerve.setFieldRelativeSpeeds(
            new ChassisSpeeds(
                speeds.getX(),
                speeds.getY(),
                rotation));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
