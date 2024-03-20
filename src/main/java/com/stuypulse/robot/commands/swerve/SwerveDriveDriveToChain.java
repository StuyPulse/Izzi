/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveToChain extends Command {

    private final Odometry odometry;
    private final SwerveDrive swerve;

    private Pose2d trapPose;

    public SwerveDriveDriveToChain() {
        odometry = Odometry.getInstance();
        swerve = SwerveDrive.getInstance();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        trapPose = Field.getClosestAllianceTrapPose(odometry.getPose());
    }

    @Override
    public void execute() {
        Rotation2d translationAngle = trapPose.getTranslation().minus(odometry.getPose().getTranslation()).getAngle();
        Translation2d translation = new Translation2d(Alignment.INTO_CHAIN_SPEED.get(), translationAngle);

        swerve.setFieldRelativeSpeeds(new ChassisSpeeds(translation.getX(), translation.getY(), 0));
    }

    private double getDistanceToTrap() {
        return odometry.getPose().getTranslation().minus(trapPose.getTranslation()).getNorm();
    }

    @Override
    public boolean isFinished() {
        return false;
        // return getDistanceToTrap() <= Alignment.TRAP_CLIMB_DISTANCE.get();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
