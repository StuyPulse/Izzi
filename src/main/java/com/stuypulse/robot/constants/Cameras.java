package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.AprilTagCamera;

import edu.wpi.first.math.geometry.Pose3d;

public interface Cameras {
    
    public CameraConfig[] APRILTAG_CAMERAS = new CameraConfig[] {
        new CameraConfig(null, null),
        new CameraConfig(null, null)
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
