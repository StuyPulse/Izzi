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

    public interface Amper {
        // TODO: determine all of these values, temporarily set to 0
        int SCORE = 31;
        int LIFT = 30;

        int ALIGNED_SWITCH_CHANNEL = 3 ;
        int MIN_LIFT_CHANNEL = 4 ;
        int MAX_LIFT_CHANNEL = 0 ; //change
        int AMP_IR_CHANNEL = 2;

    }
}
