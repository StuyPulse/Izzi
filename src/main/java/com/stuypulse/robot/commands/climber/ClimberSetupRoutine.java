/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.commands.amper.AmperToHeight;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveToChain;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimberSetupRoutine extends SequentialCommandGroup {

    public ClimberSetupRoutine() {
        addCommands(
            // raise everything and get in position
            new ParallelCommandGroup(
                new AmperToHeight(Settings.Amper.Lift.MIN_HEIGHT),
                SwerveDriveToPose.toClimb()
            ),
            // drive into chain
            new ClimberToTop(),
            new SwerveDriveDriveToChain()
        );
    }
}
