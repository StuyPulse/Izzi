/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;

public class ShooterPodiumShot extends ShooterSetRPM {

    public ShooterPodiumShot() {
        super(Settings.Shooter.PODIUM_SHOT_LEFT_RPM, Settings.Shooter.PODIUM_SHOT_RIGHT_RPM);
    }
}
