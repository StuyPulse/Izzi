/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

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

        // int TOP_RIGHT_LIMIT = 9;
        // int TOP_LEFT_LIMIT = 3;
        int BOTTOM_RIGHT_LIMIT = 9;
        int BOTTOM_LEFT_LIMIT = 4;
    }

    public interface Amper {
        int SCORE = 31;
        int LIFT = 30;

        // int ALIGNED_BUMP_SWITCH = 3;
        int LIFT_BOTTOM_LIMIT = 6;
        // int LIFT_TOP_LIMIT = 5;
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

        public interface BackLeft {
            int DRIVE = 14;
            int TURN = 17;
            int ENCODER = 3;
        }

        public interface BackRight {
            int DRIVE = 16;
            int TURN = 15;
            int ENCODER = 4;
        }
    }

    public interface Intake {
        int INTAKE_MOTOR = 40;
        int CONVEYOR_MOTOR = 41;
        int IR_SENSOR = 1;
    }

    public interface Shooter {
        int LEFT_MOTOR = 21;
        int RIGHT_MOTOR = 20;

        int IR_SENSOR = 0;
    }

    public interface LEDController {
        int PORT = 0; // PWM
    }
}
