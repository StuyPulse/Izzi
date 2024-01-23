package com.stuypulse.robot.util;

import static com.stuypulse.robot.constants.Cameras.Limelight.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.NoteDetection;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight {
    
    private final DoubleEntry txEntry;
    private final DoubleEntry tyEntry;
    private final IntegerEntry tvEntry;

    private int llID;
    private Pose3d robotRelativePose;

    private double txData;
    private double tyData;

    private IStream xAngle;
    private BStream noteData;

    public Limelight(String tableName, Pose3d robotRelativePose) {

        this.robotRelativePose = robotRelativePose;

        for(int i = 0; i < LIMELIGHTS.length; i++) if(LIMELIGHTS[i].equals(tableName)) llID = i;

        NetworkTable limelight = NetworkTableInstance.getDefault().getTable(tableName);

        txEntry = limelight.getDoubleTopic("tx").getEntry(0.0);
        tyEntry = limelight.getDoubleTopic("ty").getEntry(0.0);
        tvEntry = limelight.getIntegerTopic("tv").getEntry(0);

        xAngle = IStream.create(() -> txData)
            .filtered(new LowPassFilter(NoteDetection.X_ANGLE_RC));
        noteData = BStream.create(() -> tvEntry.get() == 1)
            .filtered(new BDebounceRC.Both(NoteDetection.DEBOUNCE_TIME));
    }

    public boolean hasNoteData() {
        return noteData.get();
    }

    public void updateData() {
        if (tvEntry.get() == 1) {
            txData = txEntry.get();
            tyData = tyEntry.get();
        }
    }

    public Pose3d getRobotRelativePose() {
        return robotRelativePose;
    }

    public double getXAngle() {
        return xAngle.get() - Units.radiansToDegrees(POSITIONS[llID].getRotation().getZ());
    }

    public double getYAngle() {
        return tyData - Units.radiansToDegrees(POSITIONS[llID].getRotation().getY());
    }

    // limelight targets far end of note, so have to subtract half of note length
    public double getDistanceToNote() {
        Rotation2d yRotation =  Rotation2d.fromDegrees(getYAngle());
        return POSITIONS[llID].getZ() / -yRotation.getTan() + POSITIONS[llID].getX() - Field.NOTE_LENGTH / 2.0;
    }
}