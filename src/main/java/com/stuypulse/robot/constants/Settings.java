/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Amp {
        public interface Score {
            SmartNumber kP = new SmartNumber("Amp/Score/kP", 1);
            SmartNumber kI = new SmartNumber("Amp/Score/kI", 0);
            SmartNumber kD = new SmartNumber("Amp/Score/kD", 0);
        } 

        public interface Lift {
            SmartNumber kP = new SmartNumber("Amp/Lift/kP", 1);
            SmartNumber kI = new SmartNumber("Amp/Lift/kI", 0);
            SmartNumber kD = new SmartNumber("Amp/Lift/kD", 0);
        }
    }
}
