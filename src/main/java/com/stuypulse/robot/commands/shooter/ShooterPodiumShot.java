/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.commands.amper.AmperToHeight;
import com.stuypulse.robot.constants.Settings.Amper;
import com.stuypulse.robot.constants.Settings.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ShooterPodiumShot extends ParallelCommandGroup {

    public ShooterPodiumShot() {
        addCommands(
            AmperToHeight.untilDone(Amper.Lift.SHOOTING_HEIGHT),
            new ShooterSetRPM(Shooter.PODIUM_SHOT)
        );
    }
}
