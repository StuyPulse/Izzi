/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.notealignment;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveAligned;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.vision.NoteVision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDriveNoteAlignedDrive extends SwerveDriveDriveAligned {

    private final Odometry odometry;
    private final NoteVision noteVision;

    public SwerveDriveNoteAlignedDrive(Gamepad driver) {
		super(driver);

		odometry = Odometry.getInstance();
		noteVision = NoteVision.getInstance();
    }

    private Translation2d getRobotTranslation() {
        return odometry.getPose().getTranslation()
            .plus(new Translation2d(Swerve.CENTER_TO_INTAKE_FRONT, 0)
                .rotateBy(odometry.getPose().getRotation()));
    }

    @Override
    public Rotation2d getTargetAngle() {
		return noteVision.getEstimatedNotePose().minus(getRobotTranslation()).getAngle();
    }

    @Override
    public double getDistanceToTarget() {
        return noteVision.getEstimatedNotePose().getDistance(getRobotTranslation());
    }
}
