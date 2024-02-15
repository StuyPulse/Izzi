/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

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
