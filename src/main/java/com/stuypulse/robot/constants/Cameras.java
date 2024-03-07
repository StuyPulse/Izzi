/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.robot.Robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** This interface stores information about each camera. */
public interface Cameras {

    public interface Limelight {
        Pose3d TUMBLER_POSE = new Pose3d(
            new Translation3d(Units.inchesToMeters(3), 0, Units.inchesToMeters(13.75)),
            new Rotation3d(0, Math.toRadians(8), Math.toRadians(2)));

        Pose3d IZZI_POSE = new Pose3d(
                new Translation3d(Units.inchesToMeters(16.156338), 0, Units.inchesToMeters(13.833919)),
                new Rotation3d(0, Units.degreesToRadians(15), 0));

        String[] LIMELIGHTS = { "limelight" };

        int[] PORTS = {5800, 5801, 5802, 5803, 5804, 5805};

        Pose3d[] POSITIONS = switch (Robot.ROBOT) {
            case IZZI    -> new Pose3d[] { IZZI_POSE };
            case TUMBLER -> new Pose3d[] {/*  TUMBLER_POSE */};
            default      -> new Pose3d[] {/*  IZZI_POSE */};
        };
    }

    public CameraConfig[] APRILTAG_CAMERAS = switch (Robot.ROBOT) {
        case IZZI -> 
            new CameraConfig[] {
                // // 10.6.94.103
                // // 172.22.11.2:3003
                // new CameraConfig("intake_camera", new Pose3d(
                //     new Translation3d(Units.inchesToMeters(16.5) + 0.1 , Units.inchesToMeters(1.0 / 8.0), Units.inchesToMeters(16.267379)),
                //     new Rotation3d(0, Units.degreesToRadians(-30), 0)),
                //     "103",
                //     3003),
                // 10.6.94.100
                // 172.22.11.2:3000
                new CameraConfig("shooter_camera", new Pose3d(
                    new Translation3d(Units.inchesToMeters(-11.5) - 0.1, 0, Units.inchesToMeters(11.75)),
                    new Rotation3d(0, Units.degreesToRadians(-9), Units.degreesToRadians(180))),
                "100",
                3000),
                // 10.6.94.102
                // 172.22.11.2:3002
                new CameraConfig("climber_camera", new Pose3d(
                    new Translation3d(Units.inchesToMeters(2.0) - 0.1, 0, Units.inchesToMeters(23.5)),
                    new Rotation3d(0, Units.degreesToRadians(-34), Units.degreesToRadians(180))), 
                "102",
                3002),
            };
        
        case TUMBLER ->
            new CameraConfig[] {
                new CameraConfig("samera1", new Pose3d(
                    -Units.inchesToMeters(12), 0, +Units.inchesToMeters(5),
                    new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180))),
                    "101",
                    3001)
            };
            
        default ->
            new CameraConfig[] {
                new CameraConfig("samera0", new Pose3d(
                    new Translation3d(),
                    new Rotation3d()),
                    "100",
                    3000)
            };
    };

    /*** LINEAR REGRESSION ***/

    // XY Standard Deviation vs Distance
    Translation2d[] xyStdDevs = new Translation2d[] {
        new Translation2d(0.5, 0.001368361309),
        new Translation2d(1, 0.001890508681),
        new Translation2d(1.5, 0.003221746028),
        new Translation2d(2, 0.009352868105),
        new Translation2d(2.5, 0.009364899366),
        new Translation2d(3, 0.01467209516),
        new Translation2d(3.5, 0.01837679393),
        new Translation2d(4, 0.03000858409),
        new Translation2d(4.5, 0.03192817984)
    };

    // Theta Standard Deviation vs Distance
    Translation2d[] thetaStdDevs = new Translation2d[] {
        new Translation2d(0.5, 0.2641393115),
        new Translation2d(1, 0.4433426481),
        new Translation2d(1.5, 0.660331025),
        new Translation2d(2, 0.6924061873),
        new Translation2d(2.5, 4.624662415),
        new Translation2d(3, 8.000007273),
        new Translation2d(3.5, 6.39384055),
        new Translation2d(4, 9.670544639),
        new Translation2d(4.5, 7.576406229)
    };

    public static class CameraConfig {
        private String name;
        private Pose3d location;
        private String ip;
        private int forwardedPort;

        public CameraConfig(String name, Pose3d location, String ip, int port) {
            this.name = name;
            this.location = location;
            this.ip = ip;
            this.forwardedPort = port;
        }

        public String getName() {
            return name;
        }

        public Pose3d getLocation() {
            return location;
        }

        public String getIP() {
            return ip;
        }

        public int getForwardedPort() {
            return forwardedPort;
        }
    }
}
