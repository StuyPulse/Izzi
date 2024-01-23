/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    double DT = 0.02;
    double WIDTH = Units.inchesToMeters(26.504);
    double LENGTH = Units.inchesToMeters(20.508);

    public interface Swerve {
        SmartNumber MAX_MODULE_SPEED = new SmartNumber("Swerve/Max Module Speed (meter per s)", 5.0);
        SmartNumber MAX_TURNING = new SmartNumber("Swerve/Max Turn Velocity (rad per s)", 6.28);

        SmartNumber MODULE_VELOCITY_DEADBAND = new SmartNumber("Swerve/Module Velocity Deadband (m per s)", 0.02);

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(3);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 1.0 / 4.71; //TODO: Find Gear Ratio

                //TODO: CHECK THESE
                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }

            public interface Turn {
                //TODO: CHECK THESE
                double POSITION_CONVERSION = 1;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }

        public interface Controller {
            //TODO: CHECK THESE
            public interface Turn {
                SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 1.0);
                SmartNumber kI = new SmartNumber("Swerve/Turn/kI", 0.0);
                SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.0);

                SmartNumber kS = new SmartNumber("Swerve/Turn/kS", 0.25);
                SmartNumber kV = new SmartNumber("Swerve/Turn/kV", 0.00);
            }

        public interface Drive {
                SmartNumber kP = new SmartNumber("Swerve/Drive/kP", 0.10);
                SmartNumber kI = new SmartNumber("Swerve/Drive/kI", 0.00);
                SmartNumber kD = new SmartNumber("Swerve/Drive/kD", 0.00);

                SmartNumber kS = new SmartNumber("Swerve/Drive/kS", 0.25);
                SmartNumber kV = new SmartNumber("Swerve/Drive/kV", 0.00);
                SmartNumber kA = new SmartNumber("Swerve/Drive/kA", 0.00);
            }
        }

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0)     
                .plus(Rotation2d.fromDegrees(0));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0)
                .plus(Rotation2d.fromDegrees(270));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0)
                .plus(Rotation2d.fromDegrees(180));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0) 
                .plus(Rotation2d.fromDegrees(90));
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
        }
    }

    public interface NoteDetection {
        SmartNumber X_ANGLE_RC = new SmartNumber("Note Detection/X Angle RC", 0.05);
        SmartNumber DEBOUNCE_TIME = new SmartNumber("Note Detection/Debounce Time", 0.15);

        SmartNumber THRESHOLD_X = new SmartNumber("Note Detection/X Threshold", 0.2);
        SmartNumber THRESHOLD_Y = new SmartNumber("Note Detection/Y Threshold", Units.inchesToMeters(2));
        SmartNumber THRESHOLD_ANGLE = new SmartNumber("Note Detection/Angle Threshold", 1);

        SmartNumber DRIVE_SPEED = new SmartNumber("Note Detection/Drive Speed", 1);        

        public interface Translation {
            SmartNumber P = new SmartNumber("Note Detection/Translation/kP", 4.0);
            SmartNumber I = new SmartNumber("Note Detection/Translation/kI", 0.0);
            SmartNumber D = new SmartNumber("Note Detection/Translation/kD", 0.15);
        }
        
        public interface Rotation {
            SmartNumber P = new SmartNumber("Note Detection/Rotation/kP", 3.5);
            SmartNumber I = new SmartNumber("Note Detection/Rotation/kI", 0.0);
            SmartNumber D = new SmartNumber("Note Detection/Rotation/kD", 0.1);
        }
    }
}
