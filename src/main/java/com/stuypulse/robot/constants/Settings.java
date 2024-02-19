/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public enum RobotType {
        JIM("03262B9F"),
        TUMBLER("0305A69D"),
        IZZI("032B4BC2"),
        SIM("");

        public final String serialNum;

        RobotType(String serialNum) {
            this.serialNum = serialNum;
        }

        public static RobotType fromString(String serialNum) {
            for (RobotType robot : RobotType.values()) {
                if (robot.serialNum.equals(serialNum.toUpperCase())) {
                    return robot;
                }
            }

            return RobotType.SIM;
        }
    }

    double DT = 1.0 / 50.0;

    double WIDTH = Units.inchesToMeters(32);
    double LENGTH = Units.inchesToMeters(36);

    public interface Climber {
        double MIN_HEIGHT = 0.0;
        double MAX_HEIGHT = Units.inchesToMeters(17.75);

        double MASS = Units.lbsToKilograms(2.173979);

        public interface BangBang {
            double CONTROLLER_VOLTAGE = 7.0;
            double THRESHOLD = 0.05;
        }

        public interface Encoder {
            double GEAR_RATIO = 1.0 / 12.0;

            // distance from center of sprocket to hook
            double SPROCKET_RADIUS = Units.inchesToMeters(0.95);
            double SPROCKET_CIRCUMFERENCE = SPROCKET_RADIUS * Math.PI * 2;

            double POSITION_CONVERSION = SPROCKET_CIRCUMFERENCE * GEAR_RATIO;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
        }
    }

    public interface Amper {
        double AMP_ROLLER_DIAMETER = Units.inchesToMeters(1.25);

        public interface Score {
            SmartNumber SCORE_SPEED = new SmartNumber("Amper/Score/Score Speed", 1.0);
            double FROM_CONVEYOR_SPEED = 0.5;
            SmartNumber TO_CONVEYOR_SPEED = new SmartNumber("Amper/Score/To Conveyor Speed", 1.0);

            double SCORE_MOTOR_CONVERSION = AMP_ROLLER_DIAMETER * Math.PI;
        }

        public interface Lift {
            double CARRIAGE_MASS = 10; // kg

            double MIN_HEIGHT = 0;
            double MAX_HEIGHT = Units.inchesToMeters(14.75);

            double VISUALIZATION_MIN_LENGTH = 0.5;
            Rotation2d ANGLE_TO_GROUND = Rotation2d.fromDegrees(68.02);

            double MAX_HEIGHT_ERROR = 0.03;

            double VEL_LIMIT = 3.0;
            double ACCEL_LIMIT = 2.0;

            // TODO: Tune these values
            SmartNumber AMP_SCORE_HEIGHT = new SmartNumber("Amper/Lift/Amp Score Height", 0.34);
            SmartNumber TRAP_SCORE_HEIGHT = new SmartNumber("Amper/Lift/Trap Score Height", 0.0);

            public interface Encoder {
                double GEARING = 1.0 / 9.0;
                double DRUM_RADIUS = Units.inchesToMeters(1.0);
                double DRUM_CIRCUMFERENCE = DRUM_RADIUS * Math.PI * 2;

                double POSITION_CONVERSION = GEARING * DRUM_CIRCUMFERENCE;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }

            public interface Feedforward {
                double kS = 0.18665;
                double kV = 6.6702;
                double kA = 0.803;
                double kG = 0.15;
            }

            public interface PID {
                double kP = 0.5;
                double kI = 0.0;
                double kD = 0.0;
            }
        }
    }

    public interface Swerve {
        // between wheel centers
        double WIDTH = Units.inchesToMeters(20.75);
        double LENGTH = Units.inchesToMeters(20.75);
        double CENTER_TO_INTAKE_FRONT = Units.inchesToMeters(17.9);

        double MAX_MODULE_SPEED = 5.0;

        double MODULE_VELOCITY_DEADBAND = 0.05;

        public interface Assist {
            SmartNumber ALIGN_MIN_SPEAKER_DIST = new SmartNumber("SwerveAssist/Minimum Distance to Speaker", 4); //change
            
            double BUZZ_INTENSITY = 0.5;

            SmartNumber kP = new SmartNumber("SwerveAssist/kP", 3.0);
            SmartNumber kI = new SmartNumber("SwerveAssist/kI", 0.0);
            SmartNumber kD = new SmartNumber("SwerveAssist/kD", 0.0);

            double ANGLE_DERIV_RC = 0.05;
            double REDUCED_FF_DIST = 0.75;
        }

        // TODO: Tune these values
        public interface Motion {
            SmartNumber MAX_VELOCITY = new SmartNumber("Swerve/Motion/Max Velocity", 3.0);
            SmartNumber MAX_ACCELERATION = new SmartNumber("Swerve/Motion/Max Acceleration", 6.0);
            SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Swerve/Motion/Max Angular Velocity", Units.degreesToRadians(540));
            SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Swerve/Motion/Max Angular Acceleration", Units.degreesToRadians(720));

            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY.get(),
                    MAX_ACCELERATION.get(),
                    MAX_ANGULAR_VELOCITY.get(),
                    MAX_ANGULAR_ACCELERATION.get());

            PIDConstants XY = new PIDConstants(0.7, 0, 0.02);
            PIDConstants THETA = new PIDConstants(10, 0, 0.1);
        }

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4);
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
            double kP = 7.0;
            double kI = 0.0;
            double kD = 0.05;

            double kS = 0.25582;
            double kV = 0.00205;
            double kA = 0.00020123;
        }

        // TODO: retune on carpet
        public interface Drive {
            SmartNumber kP = new SmartNumber("Swerve/Drive/PID/kP", 0.48346);
            double kI = 0.0;
            SmartNumber kD = new SmartNumber("Swerve/Drive/PID/kD", 0.0);

            SmartNumber kS = new SmartNumber("Swerve/Drive/FF/kS", 0.062097);
            SmartNumber kV = new SmartNumber("Swerve/Drive/FF/kV", 1.768);
            SmartNumber kA = new SmartNumber("Swerve/Drive/FF/kA", 0.41581);
        }

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(152.6);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(-48.9);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(99.4);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(179.0);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
        }
    }

    public interface NoteDetection {
        double X_ANGLE_RC = 0.05;

        SmartNumber HAS_NOTE_DEBOUNCE = new SmartNumber("Note Detection/Has Note Debounce", 0.2);

        SmartNumber THRESHOLD_X = new SmartNumber("Note Detection/X Threshold", 0.2);
        SmartNumber THRESHOLD_Y = new SmartNumber("Note Detection/Y Threshold", Units.inchesToMeters(2));
        SmartNumber THRESHOLD_ANGLE = new SmartNumber("Note Detection/Angle Threshold", 1);

        SmartNumber DRIVE_SPEED = new SmartNumber("Note Detection/Drive Speed", 1);
    }

    public interface Driver {
        public interface Drive {
            double BUZZ_DURATION = 0.2;
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.03);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.01);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.MAX_MODULE_SPEED);
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", 15);
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURNING = new SmartNumber("Driver Settings/Turn/Max Turning", 6.0);
        }
    }

    public interface Operator {
        SmartNumber DEADBAND = new SmartNumber("Operator Settings/Manual Climb + Lift Deadband", 0.05);
        SmartNumber CLIMB_DRIVE_VOLTAGE = new SmartNumber("Operator Settings/Climber Max Drive Voltage", 1.0);
        SmartNumber LIFT_DRIVE_VOLTAGE = new SmartNumber("Operator Settings/Lift Max Drive Voltage", 6.0);
        SmartNumber LIFT_ADJUST_SPEED = new SmartNumber("Operator Settings/Lift Fine Adjust Speed", Units.inchesToMeters(1.0));
    }

    public interface Shooter {
        double MOMENT_OF_INERTIA = 1;

        // TODO: Tune these values
        SmartNumber PODIUM_SHOT_LEFT_RPM = new SmartNumber("Shooter/Podium Shot Left RPM", 3750);
        SmartNumber PODIUM_SHOT_RIGHT_RPM = new SmartNumber("Shooter/Podium Shot Right RPM", 3750);

        double AMP_LEFT_RPM = PODIUM_SHOT_LEFT_RPM.get();
        double AMP_RIGHT_RPM = PODIUM_SHOT_RIGHT_RPM.get();

        SmartNumber BACKWARDS_LEFT_RPM = new SmartNumber("Shooter/Backwards Left RPM", 0);
        SmartNumber BACKWARDS_RIGHT_RPM = new SmartNumber("Shooter/Backwards Right RPM", 0);

        public interface Feedforward {
            double kV = 0.0017898;
            double kA = 0.00020903;
            double kS = 0.13793;
        }

        public interface PID {
            double kP = 0.0002;
            double kI = 0.0;
            double kD = 0.0;
        }
    }

    public interface Intake {
        public interface Detection {
            double TRIGGER_TIME = 0.0;
            double STALL_TIME = 0.05;
            double STALL_CURRENT = 40;
        }

        double ACQUIRE_SPEED = 1.0;
        double DEACQUIRE_SPEED = 1.0;
        SmartNumber MOVE_SPEED = new SmartNumber("Intake/Move Speed", 0.75);
    }

    public interface Conveyor {
        SmartNumber GANDALF_SHOOTER_SPEED = new SmartNumber("Conveyor/Gandalf Shooter Speed", 1.0);
        double GANDALF_AMP_SPEED = 1.0;
        SmartNumber FEEDER_SHOOTER_SPEED = new SmartNumber("Conveyor/Shooter Feeder To Shooter Speed", 0.5);
        SmartNumber FEEDER_AMP_SPEED = new SmartNumber("Conveyor/Shooter Feeder To Amp Speed", 1.0);

        SmartNumber DEBOUNCE_TIME = new SmartNumber("Conveyor/Debounce Time", 0.0);
        SmartNumber RECALL_DEBOUNCE = new SmartNumber("Conveyor/Recall Delay", 1.0);

        SmartNumber SHOOT_WAIT_DELAY = new SmartNumber("Conveyor/Shoot Wait Delay", 0.5);
    }

    public interface Alignment {
        double DEBOUNCE_TIME = 0.05;

        // TODO: Tune these values
        SmartNumber X_TOLERANCE = new SmartNumber("Alignment/X Tolerance", 0.1);
        SmartNumber Y_TOLERANCE = new SmartNumber("Alignment/Y Tolerance", 0.1);
        SmartNumber ANGLE_TOLERANCE = new SmartNumber("Alignment/Angle Tolerance", 5);

        // TODO: Measure these values
        SmartNumber PODIUM_SHOT_DISTANCE = new SmartNumber("Alignment/Podium Shot Distance", Units.inchesToMeters(110));
        SmartNumber PODIUM_SHOT_MAX_ANGLE = new SmartNumber("Alignment/Podium Shot Max Angle", 80);

        SmartNumber AMP_WALL_SETUP_DISTANCE = new SmartNumber("Alignment/Amp/Setup Pose Distance to Wall", Units.inchesToMeters(12.0));
        SmartNumber AMP_WALL_SCORE_DISTANCE = new SmartNumber("Alignment/Amp/Score Pose Distance to Wall", Units.inchesToMeters(6.0));

        SmartNumber TRAP_SETUP_DISTANCE = new SmartNumber("Alignment/Trap/Setup Pose Distance", Units.inchesToMeters(22.0));
        SmartNumber TRAP_CLIMB_DISTANCE = new SmartNumber("Alignment/Trap/Climb Distance", Units.inchesToMeters(18.0));

        SmartNumber INTO_CHAIN_SPEED = new SmartNumber("Alignment/Trap/Into Chain Speed", 0.25);

        // TODO: Tune these values
        public interface Translation {
            SmartNumber kP = new SmartNumber("Alignment/Translation/kP", 4.0);
            SmartNumber kI = new SmartNumber("Alignment/Translation/kI", 0.0);
            SmartNumber kD = new SmartNumber("Alignment/Translation/kD", 0.0);
        }

        // TODO: Tune these values
        public interface Rotation {
            SmartNumber kP = new SmartNumber("Alignment/Rotation/kP", 3.0);
            SmartNumber kI = new SmartNumber("Alignment/Rotation/kI", 0.0);
            SmartNumber kD = new SmartNumber("Alignment/Rotation/kD", 0.0);
        }
    }

    public interface LED {
        int LED_LENGTH = 55; // TODO: Update to Izzi
        SmartNumber BLINK_TIME = new SmartNumber("LED/LED Blink Time", .5);

        SmartNumber TRANSLATION_SPREAD = new SmartNumber("LED/LED Translation Spread (m)", 1);
        SmartNumber ROTATION_SPREAD = new SmartNumber("LED/LED Rotation Spread (deg)", 35);

        SmartBoolean LED_AUTON_TOGGLE = new SmartBoolean("LED/Auton Align Display?", true);
    }

    public static Vector2D vpow(Vector2D vec, double power) {
        return vec.mul(Math.pow(vec.magnitude(), power - 1));
    }
}
