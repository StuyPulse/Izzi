package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * This interface stores information about each camera.
 */
public interface Cameras {

    public interface Limelight {
        String [] LIMELIGHTS = {
            "limelight"
        };

        int[] PORTS = { 5800, 5801, 5802, 5803, 5804, 5805 };

        Pose3d [] POSITIONS = new Pose3d[] {
            new Pose3d(
                new Translation3d(Units.inchesToMeters(0), 0, Units.inchesToMeters(0)),
                new Rotation3d(0, Math.toRadians(0), Math.toRadians(0))
            )
        };
    }
    
    public CameraConfig[] APRILTAG_CAMERAS = new CameraConfig[] {
        // TODO: Update with real values
        new CameraConfig("samera0", new Pose3d(new Translation3d(), new Rotation3d())),
        new CameraConfig("samera1", new Pose3d(new Translation3d(), new Rotation3d())),
        new CameraConfig("samera2", new Pose3d(new Translation3d(), new Rotation3d())),
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
