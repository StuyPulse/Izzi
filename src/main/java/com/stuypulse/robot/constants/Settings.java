/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Swerve {
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
    }

}
