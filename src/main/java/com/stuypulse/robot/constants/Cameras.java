/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.robot.Robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** This interface stores information about each camera. */
public interface Cameras {

    public interface Limelight {
        Pose3d TUMBLER_POSE = new Pose3d(
            new Translation3d(Units.inchesToMeters(3), 0, Units.inchesToMeters(13.75)),
            new Rotation3d(0, Math.toRadians(8), Math.toRadians(2)));

        Pose3d IZZI_POSE = new Pose3d(
            new Translation3d(0, Units.inchesToMeters(16.156338), Units.inchesToMeters(13.833919)),
            new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)))

        String[] LIMELIGHTS = {"limelight"};

        int[] PORTS = {5800, 5801, 5802, 5803, 5804, 5805};

        Pose3d[] POSITIONS = switch (Robot.ROBOT) {
            case IZZI    -> new Pose3d[] { IZZI_POSE };
            case TUMBLER -> new Pose3d[] { TUMBLER_POSE };
            default      -> new Pose3d[] { IZZI_POSE };
        };
    }

    public CameraConfig[] APRILTAG_CAMERAS = switch (Robot.ROBOT) {
        case IZZI -> 
            new CameraConfig[] {
                new CameraConfig("samera0", new Pose3d(new Translation3d(0, Units.inchesToMeters(16.387490), Units.inchesToMeters(16.267379)), new Rotation3d(0, Units.degreesToRadians(60), 0))),                               // INTAKE
                new CameraConfig("samera1", new Pose3d(new Translation3d(0, -Units.inchesToMeters(11.59286), Units.inchesToMeters(11.926806)), new Rotation3d(0, Units.degreesToRadians(-56), Units.degreesToRadians(180)))),    // SHOOTER
                new CameraConfig("samera2", new Pose3d(new Translation3d(0, Units.inchesToMeters(2.307881), Units.inchesToMeters(23.974534)), new Rotation3d(0, Units.degreesToRadians(-81), Units.degreesToRadians(180)))),     // CLIMBER
            };
        
        case TUMBLER ->
            new CameraConfig[] {
                new CameraConfig("samera1",
                    new Pose3d(
                        -Units.inchesToMeters(12), 0, +Units.inchesToMeters(5),
                        new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180))))
            };
            
        default -> new CameraConfig[] {
            new CameraConfig("samera0", new Pose3d(new Translation3d(), new Rotation3d()))
        };
    };

    public static class CameraConfig {
        private String name;
        private Pose3d location;

        public CameraConfig(String name, Pose3d location) {
            this.name = name;
            this.location = location;
        }

        public String getName() {
            return name;
        }

        public Pose3d getLocation() {
            return location;
        }
    }
}
