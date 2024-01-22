/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface Climber {
        int LEFT_MOTOR = 3;
        int RIGHT_MOTOR = 4; // placeholders for now

        int TOP_RIGHT_LIMIT = 0;
        int TOP_LEFT_LIMIT = 0;
        int BOTTOM_RIGHT_LIMIT = 0;
        int BOTTOM_LEFT_LIMIT = 0;        
    }
}
