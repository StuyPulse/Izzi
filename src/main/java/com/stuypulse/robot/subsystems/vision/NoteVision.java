package com.stuypulse.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class NoteVision extends SubsystemBase {

    private static final NoteVision instance;

    static {
        instance = new LLNoteVision();
    }

    public static NoteVision getInstance() {
        return instance;
    }

    public abstract boolean hasNoteData();

    public abstract Translation2d getEstimatedNotePose();
    public abstract Translation2d getRobotRelativeNotePose();

    public final Rotation2d getRotationToNote() {
        return getRobotRelativeNotePose().getAngle();
    }

}
