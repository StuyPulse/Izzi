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

    public interface Conveyor {

        SmartNumber GANDALF_SHOOTER_SPEED = new SmartNumber("Conveyor/Gandalf Shooter Speed", 1);
        SmartNumber GANDALF_AMP_SPEED = new SmartNumber("Conveyor/Gandalf Amp Speed", -1);
        SmartNumber SHOOTER_FEEDER_SPEED = new SmartNumber("Conveyor/Shooter Feeder Speed", 1);
        
        SmartNumber DEBOUNCE_TIME = new SmartNumber("Conveyor/Debounce Time", 0.2);
    }

}

