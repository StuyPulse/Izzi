/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.util.ShooterSpeeds;

public class ShooterStop extends ShooterSetRPM {

    public ShooterStop() {
        super(new ShooterSpeeds());
    }
}
