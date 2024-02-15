/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;

public class ClimberToBottom extends ClimberToHeight {
    public ClimberToBottom() {
        super(Settings.Climber.MIN_HEIGHT);
    }
}
