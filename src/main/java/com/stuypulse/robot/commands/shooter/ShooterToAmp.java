/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;

public class ShooterToAmp extends ShooterSetRPM {

    public ShooterToAmp() {
        super(Settings.Shooter.AMP_LEFT_RPM, Settings.Shooter.AMP_RIGHT_RPM);
    }
}
