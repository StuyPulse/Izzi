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
        int SCORE = 31;
        int LIFT = 30;

        int ALIGNED_SWITCH_CHANNEL = 3 ;
        int MIN_LIFT_CHANNEL = 4 ;
        int AMP_IR_CHANNEL = 2;
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
        int SENSOR = 1;
    }
  
    public interface Shooter {
        int LEFT_MOTOR = 20; 
        int RIGHT_MOTOR = 21;
    }

    public interface Conveyor {
        int GANDALF_MOTOR_PORT = 50;
        int SHOOTER_FEEDER_MOTOR_PORT = 51;

        int IR_SENSOR_PORT = 0;
    }
}
