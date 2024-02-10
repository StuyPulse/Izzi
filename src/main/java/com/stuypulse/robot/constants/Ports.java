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
        int LEFT_MOTOR = 60;
        int RIGHT_MOTOR = 61;

        int TOP_RIGHT_LIMIT = 9;
        int TOP_LEFT_LIMIT = 8;
        int BOTTOM_RIGHT_LIMIT = 7;
        int BOTTOM_LEFT_LIMIT = 6;        
    }
  
    public interface Amper {
        int SCORE = 31;
        int LIFT = 30;

        // int ALIGNED_BUMP_SWITCH = 3;
        int LIFT_BOTTOM_LIMIT = 4;
        int LIFT_TOP_LIMIT = 5;
        int AMP_IR = 2;
    }
  
    public interface Swerve {
        public interface FrontRight {
            int DRIVE = 10;
            int TURN = 11;
            int ENCODER = 1;
        }

        public interface FrontLeft {
            int DRIVE = 12;
            int TURN = 13;
            int ENCODER = 2;
        }

        public interface BackLeft{
            int DRIVE = 14;
            int TURN = 15;
            int ENCODER = 3;
        }

        public interface BackRight {
            int DRIVE = 16;
            int TURN = 17;
            int ENCODER = 4;
        }
    }

    public interface Intake {
        int MOTOR = 40;
        int IR_SENSOR = 1;
    }
  
    public interface Shooter {
        int LEFT_MOTOR = 20; 
        int RIGHT_MOTOR = 21;
    }

    public interface Conveyor {
        int GANDALF = 50;
        int FEEDER = 51;

        int IR_SENSOR = 0;
    }
}
