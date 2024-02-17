/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** This interface stores information about each camera. */
public interface Cameras {

    public interface Limelight {
        String[] LIMELIGHTS = {"limelight"};

        int[] PORTS = {5800, 5801, 5802, 5803, 5804, 5805};

        Pose3d[] POSITIONS =
            new Pose3d[] {
                new Pose3d(
                    new Translation3d(Units.inchesToMeters(0), 0, Units.inchesToMeters(0)),
                    new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)))
            };
    }

    public CameraConfig[] APRILTAG_CAMERAS =
        new CameraConfig[] {
            // TODO: Update with real values
            new CameraConfig("samera0", new Pose3d(new Translation3d(0, 16.387490, 16.267379), new Rotation3d(0, 0, 0))),     // INTAKE
            new CameraConfig("samera1", new Pose3d(new Translation3d(0, -11.59286, 11.926806), new Rotation3d(0, 0, 180))),    // SHOOTER
            new CameraConfig("samera2", new Pose3d(new Translation3d(0, 2.307881, 23.974534), new Rotation3d(0, 0, 180))),     // CLIMBER
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
