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

    double DT = 0.02; // time between each simulation update
    
    public interface Amper {
        
        public interface Score {
            double ROLLER_SPEED = 1.0; // change later
        }
        
        public interface Lift {
           
            double GEARING = 9; // ~9:1
            double CARRIAGE_MASS = 10; // kg
            double DRUM_RADIUS = 0.025; // meters 
            double ENCODER_CONVERSION = 0; // determine;

            double MIN_HEIGHT = 0;
            double MAX_HEIGHT = 1.8475325; // meters 

            double MAX_HEIGHT_ERROR = 0.03;

            SmartNumber VEL_LIMIT = new SmartNumber("Velocity Limit", 3);
            SmartNumber ACC_LIMIT = new SmartNumber("Acceleration Limit", 2);
                
            SmartNumber kP = new SmartNumber("Amp/Lift/kP", 1);
            SmartNumber kI = new SmartNumber("Amp/Lift/kI", 0);
            SmartNumber kD = new SmartNumber("Amp/Lift/kD", 0);
        }
    }
}
