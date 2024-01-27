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
    double DT = 0.02;

    public interface Climber {
        double MIN_HEIGHT = 0.0;
        double MAX_HEIGHT = 1.0;

        double MASS = 10.0;
        double DRUM_RADIUS = 0.025;

        SmartNumber VELOCITY_LIMIT = new SmartNumber("Climber/Velocity Limit", 3.0); 

        public interface Encoder {
            double VOLTAGE = 1.0;
            double THRESHOLD = 0.03;
            double GEAR_RATIO = 9.0;

            double POSITION_CONVERSION = 1.0;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
        }
    }

    public interface Operator {
        SmartNumber DEADBAND = new SmartNumber("Operator/Deadband", 0.02);
    }
}