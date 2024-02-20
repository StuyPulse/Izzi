/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class NoteVision extends SubsystemBase {

    private static final NoteVision instance;

    static {
        instance = new LLNoteVision();
    }

    public static NoteVision getInstance() {
        return instance;
    }

    public final boolean withinIntakePath() {
        if (!hasNoteData()) return false;

        Translation2d robotRelative = getRobotRelativeNotePose();

        return robotRelative.getX() > 0
            && robotRelative.getX() < Settings.NoteDetection.INTAKE_THRESHOLD_DISTANCE.get()
            && Math.abs(robotRelative.getY()) < Settings.Swerve.WIDTH / 2.0;
    }

    public abstract boolean hasNoteData();

    public abstract Translation2d getEstimatedNotePose();

    public abstract Translation2d getRobotRelativeNotePose();

    public final Rotation2d getRotationToNote() {
        return getRobotRelativeNotePose().getAngle();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Note Detection/Has Note Data", hasNoteData());
        SmartDashboard.putBoolean("Note Detection/Is in Intake Path", withinIntakePath());
    }
}
