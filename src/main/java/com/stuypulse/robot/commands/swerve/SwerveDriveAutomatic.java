/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Driver;
import com.stuypulse.robot.constants.Settings.Swerve.*;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.vision.NoteVision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDriveAutomatic extends SwerveDriveDriveAligned {

    private final Intake intake;
    private final Conveyor conveyor;
    private final Amper amper;
    private final Odometry odometry;
    
    private final NoteVision llNoteVision;
    
    private final Gamepad driver;

    public SwerveDriveAutomatic(Gamepad driver) {
        super(driver);

        this.driver = driver;

        odometry = Odometry.getInstance();
        intake = Intake.getInstance();
        conveyor = Conveyor.getInstance();
        amper = Amper.getInstance();
        llNoteVision = NoteVision.getInstance();
    }

    @Override
    public Rotation2d getTargetAngle() {
        // if note in amp face wall
        if (amper.hasNote()) {
            return Rotation2d.fromDegrees(Robot.isBlue() ? 270.0 : 90.0);
        }

        Translation2d robotPose = odometry.getPose().getTranslation();
        
        Translation2d speakerPose = Field.getAllianceSpeakerPose().getTranslation();

        double distanceToSpeaker = speakerPose.getDistance(robotPose);

        if ((intake.hasNote())
                && (distanceToSpeaker < Assist.ALIGN_MIN_SPEAKER_DIST.get())) {
            return speakerPose.minus(robotPose).getAngle()
                .plus(Rotation2d.fromDegrees(180));
        }
        
        if (llNoteVision.hasNoteData()) {
            return llNoteVision.getRobotRelativeNotePose().getAngle();
        }

        return odometry.getPose().getRotation();
    }

    @Override
    public double getDistanceToTarget() {
        if (amper.hasNote()) return 100;

        Translation2d robotPose = odometry.getPose().getTranslation();
        
        Translation2d speakerPose = Field.getAllianceSpeakerPose().getTranslation();

        double distanceToSpeaker = speakerPose.getDistance(robotPose);

        if ((intake.hasNote())
                && (distanceToSpeaker < Assist.ALIGN_MIN_SPEAKER_DIST.get())) {
            return speakerPose.getDistance(robotPose);
        }
        
        if (llNoteVision.hasNoteData()) {
            return llNoteVision.getRobotRelativeNotePose().getDistance(new Translation2d());
        }

        return 0;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(driver.getRightX()) > Driver.Turn.DEADBAND.getAsDouble();
    }
}
