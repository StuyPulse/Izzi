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
    double DT = 0.02;

    public interface Shooter {
        double MOMENT_OF_INERTIA = 1;
        
        SmartNumber PODIUM_SHOT_LEFT_RPM = new SmartNumber("Shooter/Podium Shot Right RPM", 0);
        SmartNumber PODIUM_SHOT_RIGHT_RPM = new SmartNumber("Shooter/Podium Shot Left RPM", 0);
        
        public interface Feedforward {
            SmartNumber kV = new SmartNumber("Shooter/Feedforward kV",0);
            SmartNumber kA = new SmartNumber("Shooter/Feedforward kA",0);
            SmartNumber kS = new SmartNumber("Shooter/Feedforward kS",0); //CHANGE LATER            
        }

        public interface PID {
            SmartNumber kP = new SmartNumber("Shooter/PID kP", 0);
            SmartNumber kI = new SmartNumber("Shooter/PID kI", 0);
            SmartNumber kD = new SmartNumber("Shooter/PID kD", 0);
        }
    
        
    }
}
