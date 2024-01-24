/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Climber {
        double MIN_HEIGHT = 0.0;
        double MAX_HEIGHT = 0.0;

        public interface Encoder {
            double GEAR_RATIO = 0.0;

            double POSITION_CONVERSION = 0.0;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
        }
    }
}