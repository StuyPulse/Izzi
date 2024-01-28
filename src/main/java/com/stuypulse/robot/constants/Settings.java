/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import edu.wpi.first.math.util.Units;
import com.stuypulse.stuylib.math.Vector2D;
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

    double DT = 1.0 / 50.0;

    public interface Climber {
        double MIN_HEIGHT = 0.0;
        double MAX_HEIGHT = 0.0;

        double AT_HEIGHT_THRESHOLD = Units.inchesToMeters(1);

        double BANGBANG_VOLTAGE = 8;

        public interface Encoder {
            double GEAR_RATIO = 0.0;

            double POSITION_CONVERSION = 0.0;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
        }
    }  
  
    public interface Amper {
        public interface Score {
            SmartNumber ROLLER_SPEED = new SmartNumber("Amper/Score/Roller Speed", 1.0); // change later
        }
        
        public interface Lift {
            double CARRIAGE_MASS = 10; // kg

            double MIN_HEIGHT = 0;
            double MAX_HEIGHT = 1.8475325; // meters 

            double MAX_HEIGHT_ERROR = 0.03;

            SmartNumber VEL_LIMIT = new SmartNumber("Amper/Lift/Velocity Limit", 3);
            SmartNumber ACC_LIMIT = new SmartNumber("Amper/Lift/Acceleration Limit", 2);

            public interface Encoder {
                double GEARING = 9; // ~9:1
                double DRUM_RADIUS = 0.025; // meters 
                double DRUM_CIRCUMFERENCE = DRUM_RADIUS * Math.PI * 2;

                double POSITION_CONVERSION = GEARING * DRUM_CIRCUMFERENCE;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }

            public interface PID {
                SmartNumber kP = new SmartNumber("Amper/Lift/kP", 1);
                SmartNumber kI = new SmartNumber("Amper/Lift/kI", 0);
                SmartNumber kD = new SmartNumber("Amper/Lift/kD", 0);
            }
        }
    }

    public interface Swerve {
        // between wheel centers
        double WIDTH = Units.inchesToMeters(20.75);
        double LENGTH = Units.inchesToMeters(20.75);

        SmartNumber MAX_MODULE_SPEED = new SmartNumber("Swerve/Max Module Speed (meter per s)", 5.0);
        SmartNumber MAX_TURNING = new SmartNumber("Swerve/Max Turn Velocity (rad per s)", 6.28);

        SmartNumber MODULE_VELOCITY_DEADBAND = new SmartNumber("Swerve/Module Velocity Deadband (m per s)", 0.02);

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(3);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 1.0 / 4.71;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }

            public interface Turn {
                double POSITION_CONVERSION = 1;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }

        public interface Turn {
            SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 1.0);
            SmartNumber kI = new SmartNumber("Swerve/Turn/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.0);

            SmartNumber kS = new SmartNumber("Swerve/Turn/kS", 0.01);
            SmartNumber kV = new SmartNumber("Swerve/Turn/kV", 0.25);
            SmartNumber kA = new SmartNumber("Swerve/Turn/kA", 0.01);
        }

        public interface Drive {
            SmartNumber kP = new SmartNumber("Swerve/Drive/kP", 1.0);
            SmartNumber kI = new SmartNumber("Swerve/Drive/kI", 0.00);
            SmartNumber kD = new SmartNumber("Swerve/Drive/kD", 0.00);

            SmartNumber kS = new SmartNumber("Swerve/Drive/kS", 0.01);
            SmartNumber kV = new SmartNumber("Swerve/Drive/kV", 0.25);
            SmartNumber kA = new SmartNumber("Swerve/Drive/kA", 0.01);
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

    public interface Driver {
        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.03);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.01);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.MAX_MODULE_SPEED.get());
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", 15);
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.1);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURNING = new SmartNumber("Driver Settings/Turn/Max Turning", 6.0);
        }
    }

    public interface Shooter {
        double MOMENT_OF_INERTIA = 1;
        
        SmartNumber PODIUM_SHOT_LEFT_RPM = new SmartNumber("Shooter/Podium Shot Right RPM", 0);
        SmartNumber PODIUM_SHOT_RIGHT_RPM = new SmartNumber("Shooter/Podium Shot Left RPM", 0);
        
        public interface Feedforward {
            SmartNumber kV = new SmartNumber("Shooter/Feedforward/kV",0);
            SmartNumber kA = new SmartNumber("Shooter/Feedforward/kA",0);
            SmartNumber kS = new SmartNumber("Shooter/Feedforward/kS",0); //CHANGE LATER            
        }

        public interface PID {
            SmartNumber kP = new SmartNumber("Shooter/PID/kP", 0);
            SmartNumber kI = new SmartNumber("Shooter/PID/kI", 0);
            SmartNumber kD = new SmartNumber("Shooter/PID/kD", 0);
        }
    }

    public interface Intake {
        public interface Detection {
            SmartNumber TRIGGER_TIME = new SmartNumber("Intake/Trigger Debounce Time", 0.05);
            SmartNumber STALL_TIME = new SmartNumber("Intake/Stall Debounce Time", .05);
            SmartNumber STALL_CURRENT = new SmartNumber("Intake/Stall Current", 40);
        }
        SmartNumber ACQUIRE_SPEED = new SmartNumber("Intake/Acquire", 1);
        SmartNumber DEACQUIRE_SPEED = new SmartNumber("Intake/Deacquire", -1);
    }
  
    public interface Conveyor {
        SmartNumber GANDALF_SHOOTER_SPEED = new SmartNumber("Conveyor/Gandalf Shooter Speed", 1);
        SmartNumber GANDALF_AMP_SPEED = new SmartNumber("Conveyor/Gandalf Amp Speed", 1);
        SmartNumber FEEDER_SHOOTER_SPEED = new SmartNumber("Conveyor/Shooter Feeder Speed", 1);
        SmartNumber FEEDER_AMP_SPEED = new SmartNumber("Conveyor/Shooter Feeder Speed", 1);
        
        SmartNumber DEBOUNCE_TIME = new SmartNumber("Conveyor/Debounce Time", 0.2);
    }

    public static Vector2D vpow(Vector2D vec, double power) {
        return vec.mul(Math.pow(vec.magnitude(), power - 1));
    }
}
