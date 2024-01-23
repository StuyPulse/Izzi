/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Intake {
        SmartNumber TRIGGER_TIME = new SmartNumber("Intake/Trigger Time (Rising)", 0.05);
        SmartNumber ACQUIRE = new SmartNumber("Intake/Acquire", 1);
        SmartNumber DEACQUIRE = new SmartNumber("Intake/Deacquire", 1);
    }
}
