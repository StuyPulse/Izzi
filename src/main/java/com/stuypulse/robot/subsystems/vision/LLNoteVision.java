package com.stuypulse.robot.subsystems.vision;

import static com.stuypulse.robot.constants.Cameras.Limelight.*;

import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.vision.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LLNoteVision extends NoteVision {

    private final Limelight[] limelights;
    private Translation2d notePose;
    private FieldObject2d note;

    protected LLNoteVision() {
        String[] hostNames = LIMELIGHTS;

        limelights = new Limelight[hostNames.length];

        for (int i = 0; i < hostNames.length; i++) {
            limelights[i] = new Limelight(hostNames[i], POSITIONS[i]);

            for (int port : PORTS) {
                PortForwarder.add(port + i * 10, hostNames[i] + ".local", port);
            }
        }

        note = Odometry.getInstance().getField().getObject("Note");

        notePose = new Translation2d();
    }

    /**
     * Get whether the Limelight has data.
     * @return whether the Limelight has data
     */
    @Override
    public boolean hasNoteData() {
        for (Limelight limelight : limelights) {
            if (limelight.hasNoteData()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Get the estimated pose of the note.
     * @return the estimated pose of the note
     */
    @Override
    public Translation2d getEstimatedNotePose() {
        return notePose;
    }

    @Override
    public Translation2d getRobotRelativeNotePose() {
        Translation2d sum = new Translation2d();

        for (Limelight limelight : limelights) {
            sum = sum.plus(new Translation2d(limelight.getDistanceToNote(), Rotation2d.fromDegrees(limelight.getXAngle())));
        }

        return sum.div(limelights.length);
    }

    /**
     * @Calculates the estimated pose of the note by averaging data from all available limelights.
     * Sets the pose to `notePose` that can be accessed with `getEstimatedNotePose()`.
     */
    private void updateNotePose() {
        Translation2d sum = new Translation2d();

        for (Limelight limelight : limelights) {
            Odometry odometry = Odometry.getInstance();

            Translation2d limelightToNote = new Translation2d(limelight.getDistanceToNote(), Rotation2d.fromDegrees(limelight.getXAngle()));

            Translation2d robotToNote = limelightToNote
                .minus(limelight.getRobotRelativePose().getTranslation().toTranslation2d())
                .rotateBy(limelight.getRobotRelativePose().getRotation().toRotation2d());

            Translation2d fieldToNote = robotToNote.rotateBy(odometry.getPose().getRotation()).plus(odometry.getPose().getTranslation());

            sum = sum.plus(fieldToNote);
        }

        notePose = sum.div(limelights.length);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < limelights.length; ++i) {
            limelights[i].updateData();
            notePose = getEstimatedNotePose();
        }

        note.setPose(new Pose2d(notePose, new Rotation2d()));

        if (hasNoteData()) updateNotePose();
        updateTelemetry();
    }

    private void updateTelemetry() {
        if (hasNoteData()) {
            SmartDashboard.putNumber("Note Detection/X Angle", limelights[0].getXAngle());
            SmartDashboard.putNumber("Note Detection/Y Angle", limelights[0].getYAngle());
            SmartDashboard.putNumber("Note Detection/Distance", limelights[0].getDistanceToNote());
            SmartDashboard.putNumber("Note Detection/Estimated X", notePose.getX());
            SmartDashboard.putNumber("Note Detection/Estimated Y", notePose.getY());
        } else {
            SmartDashboard.putNumber("Note Detection/X Angle", 0);
            SmartDashboard.putNumber("Note Detection/Y Angle", 0);
            SmartDashboard.putNumber("Note Detection/Distance", 0);
            SmartDashboard.putNumber("Note Detection/Estimated X", 0);
            SmartDashboard.putNumber("Note Detection/Estimated Y", 0);
        }
    }
}