/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.commands.amper.AmperToHeight;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveToChain;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimberSetupRoutine extends SequentialCommandGroup {

    private final double X_TOLERANCE_INCHES = 3.0;
    private final double Y_TOLERANCE_INCHES = 3.0;
    private final double ANGLE_TOLERANCE_DEGREES = 3.0;

    private Pose2d getTargetPose() {
	Pose2d closestTrap = Field.getClosestAllianceTrapPose(Odometry.getInstance().getPose());
	Translation2d offsetTranslation = new Translation2d(Alignment.TRAP_SETUP_DISTANCE.get(), closestTrap.getRotation());

	return new Pose2d(closestTrap.getTranslation().plus(offsetTranslation), closestTrap.getRotation());
    }

    public ClimberSetupRoutine() {
        addCommands(
            // raise everything and get in position
            new ParallelCommandGroup(
                new AmperToHeight(Settings.Amper.Lift.MIN_HEIGHT),
                new SwerveDriveToPose(() -> getTargetPose())
                    .withTolerance(
                        Units.inchesToMeters(X_TOLERANCE_INCHES),
                        Units.inchesToMeters(Y_TOLERANCE_INCHES),
                        ANGLE_TOLERANCE_DEGREES)),
            // drive into chain
            new ClimberToTop(),
            new SwerveDriveDriveToChain()
        );
    }
}
