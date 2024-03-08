/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.vision.AprilTag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

import java.util.ArrayList;
import java.util.Arrays;

/** This interface stores information about the field elements. */
public interface Field {

    double WIDTH = Units.inchesToMeters(323.25);
    double LENGTH = Units.inchesToMeters(651.25);

    double NOTE_LENGTH = Units.inchesToMeters(14.0);

    public static Pose3d transformToOppositeAlliance(Pose3d pose) {
        Pose3d rotated = pose.rotateBy(new Rotation3d(0, 0, Math.PI));

        return new Pose3d(
            rotated.getTranslation().plus(new Translation3d(LENGTH, WIDTH, 0)),
            rotated.getRotation());
    }

    public static Pose2d transformToOppositeAlliance(Pose2d pose) {
        Pose2d rotated = pose.rotateBy(Rotation2d.fromDegrees(180));
        return new Pose2d(
            rotated.getTranslation().plus(new Translation2d(LENGTH, WIDTH)),
            rotated.getRotation());
    }

    /*** APRILTAGS ***/

    double APRILTAG_SIZE = Units.inchesToMeters(6.125);

    enum NamedTags {
        BLUE_SOURCE_RIGHT,
        BLUE_SOURCE_LEFT,
        RED_SPEAKER_OFFSET,
        RED_SPEAKER,
        RED_AMP,
        BLUE_AMP,
        BLUE_SPEAKER,
        BLUE_SPEAKER_OFFSET,
        RED_SOURCE_RIGHT,
        RED_SOURCE_LEFT,
        RED_STAGE_LEFT,
        RED_STAGE_RIGHT,
        RED_STAGE_FAR,
        BLUE_STAGE_FAR,
        BLUE_STAGE_LEFT,
        BLUE_STAGE_RIGHT;

        public final AprilTag tag;

        public int getID() {
            return tag.getID();
        }

        public Pose3d getLocation() {
            return tag.getLocation();
        }

        private NamedTags() {
            tag = APRILTAGS[ordinal()];
        }
    }

    AprilTag APRILTAGS[] = {
        // 2024 Field AprilTag Layout
        new AprilTag(1,  new Pose3d(new Translation3d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
        new AprilTag(2,  new Pose3d(new Translation3d(Units.inchesToMeters(637.21), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
        new AprilTag(3,  new Pose3d(new Translation3d(Units.inchesToMeters(652.73), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(4,  new Pose3d(new Translation3d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(5,  new Pose3d(new Translation3d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.0), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new AprilTag(6,  new Pose3d(new Translation3d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.0), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new AprilTag(7,  new Pose3d(new Translation3d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(8,  new Pose3d(new Translation3d(Units.inchesToMeters(-1.5), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(9,  new Pose3d(new Translation3d(Units.inchesToMeters(14.02), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
        new AprilTag(10, new Pose3d(new Translation3d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
        new AprilTag(11, new Pose3d(new Translation3d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(300)))),
        new AprilTag(12, new Pose3d(new Translation3d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.10), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(60)))),
        new AprilTag(13, new Pose3d(new Translation3d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(14, new Pose3d(new Translation3d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(15, new Pose3d(new Translation3d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.10), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(120)))),
        new AprilTag(16, new Pose3d(new Translation3d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Units.inchesToMeters(52.0)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(240)))),
    };

    public static boolean isValidTag(int id) {
        for (AprilTag tag : APRILTAGS) {
            if (tag.getID() == id) {
                return true;
            }
        }
        return false;
    }

    public static AprilTag[] getApriltagLayout(int... ids) {
        ArrayList<AprilTag> tags = new ArrayList<AprilTag>();

        for (int id : ids) {
            for (AprilTag tag : APRILTAGS) {
                if (tag.getID() == id) {
                    tags.add(tag);
                }
            }
        }

        AprilTag[] tags_array = new AprilTag[tags.size()];
        return tags.toArray(tags_array);
    }

    public static double[] getLayoutAsDoubleArray(AprilTag[] tags) {
        double[] layout = new double[tags.length * 7];
        for (int i = 0; i < tags.length; i++) {
            layout[i * 7 + 0] = tags[i].getID();
            layout[i * 7 + 1] = tags[i].getLocation().getX();
            layout[i * 7 + 2] = tags[i].getLocation().getY();
            layout[i * 7 + 3] = tags[i].getLocation().getZ();
            layout[i * 7 + 4] = tags[i].getLocation().getRotation().getX();
            layout[i * 7 + 5] = tags[i].getLocation().getRotation().getY();
            layout[i * 7 + 6] = tags[i].getLocation().getRotation().getZ();
        }
        return layout;
    }

    public static AprilTag getTag(int id) {
        for (AprilTag tag : APRILTAGS) {
            if (tag.getID() == id) {
                return tag;
            }
        }
        return null;
    }

    /*** SPEAKER ***/

    public static Pose2d getAllianceSpeakerPose() {
        return (Robot.isBlue() ? NamedTags.BLUE_SPEAKER : NamedTags.RED_SPEAKER)
            .getLocation().toPose2d();
    }

    public static Pose2d getSpeakerPathFindPose() {
        return getAllianceSpeakerPose().transformBy(
            new Transform2d(0, Units.inchesToMeters(200), new Rotation2d()));
    }

    double SPEAKER_OPENING_X = Units.inchesToMeters(13.6);

    /*** AMP ***/

    public static Pose2d getAllianceAmpPose() {
        return (Robot.isBlue() ? NamedTags.BLUE_AMP : NamedTags.RED_AMP)
            .getLocation().toPose2d();
    }

    public static Pose2d getOpposingAmpPose() {
        return (Robot.isBlue() ? NamedTags.RED_AMP : NamedTags.BLUE_AMP)
            .getLocation().toPose2d();
    }

    public static Pose2d getAmpPathFindPose() {
        return getAllianceAmpPose().transformBy(
            new Transform2d(0, Units.inchesToMeters(56), new Rotation2d()));
    }

    public static AprilTag getAllianceAmpTag() {
        return (Robot.isBlue() ? NamedTags.BLUE_AMP : NamedTags.RED_AMP).tag;
    }

    /*** TRAP ***/

    public static Pose2d[] getAllianceTrapPoses() {
        if (Robot.isBlue()) {
            return new Pose2d[] {
                NamedTags.BLUE_STAGE_FAR.getLocation().toPose2d(),
                NamedTags.BLUE_STAGE_LEFT.getLocation().toPose2d(),
                NamedTags.BLUE_STAGE_RIGHT.getLocation().toPose2d()
            };
        } else {
            return new Pose2d[] {
                NamedTags.RED_STAGE_FAR.getLocation().toPose2d(),
                NamedTags.RED_STAGE_LEFT.getLocation().toPose2d(),
                NamedTags.RED_STAGE_RIGHT.getLocation().toPose2d()
            };
        }
    }

    public static Pose2d getClosestAllianceTrapPose(Pose2d robotPose) {
        return robotPose.nearest(Arrays.asList(getAllianceTrapPoses()));
    }

    /*** STAGE ***/

    Translation2d[] CLOSE_STAGE_TRIANGLE = new Translation2d[] {
        new Translation2d(Units.inchesToMeters(125.0), WIDTH / 2.0),                       // center 
        new Translation2d(Units.inchesToMeters(222.6), Units.inchesToMeters(105)),  // bottom
        new Translation2d(Units.inchesToMeters(222.6), Units.inchesToMeters(205.9)) // top
    };

    Translation2d[] FAR_STAGE_TRIANGLE = new Translation2d[] {
        new Translation2d(Field.LENGTH - CLOSE_STAGE_TRIANGLE[0].getX(), CLOSE_STAGE_TRIANGLE[0].getY()), // center 
        new Translation2d(Field.LENGTH - CLOSE_STAGE_TRIANGLE[1].getX(), CLOSE_STAGE_TRIANGLE[1].getY()), // bottom
        new Translation2d(Field.LENGTH - CLOSE_STAGE_TRIANGLE[2].getX(), CLOSE_STAGE_TRIANGLE[2].getY()), // top
    };

    public static boolean robotUnderStage() {
        Translation2d robot = Odometry.getInstance().getPose().getTranslation();
        
        return pointInTriangle(robot, CLOSE_STAGE_TRIANGLE) || pointInTriangle(robot, FAR_STAGE_TRIANGLE);
    }

    private static boolean pointInTriangle(Translation2d point, Translation2d[] triangle) {
        double[] slopes = new double[3];
        double[] yIntercepts = new double[3];

        // Constructing lines from the triangles to check if the robot is under the stage
        for (int i = 0; i < 3; i++) {
            slopes[i] = (triangle[(i + 1) % 3].getY() - triangle[i].getY())
                    / (triangle[(i + 1) % 3].getX() - triangle[i].getX());

            yIntercepts[i] = triangle[i].getY() - slopes[i] * triangle[i].getX();
        }

        // Checking if the robot is under the stage by comparing the robot's position to the lines
        for (int i = 0; i < 3; i++) {
            if (point.getY() > slopes[i] * point.getX() + yIntercepts[i]
                    && point.getY() < Math.max(triangle[i].getY(), triangle[(i + 1) % 3].getY())
                    && point.getY() > Math.min(triangle[i].getY(), triangle[(i + 1) % 3].getY())
                    && point.getX() > Math.min(triangle[i].getX(), triangle[(i + 1) % 3].getX())
                    && point.getX() < Math.max(triangle[i].getX(), triangle[(i + 1) % 3].getX())) {
                return true;
            }
        }

        return false;
    }

    /***** NOTE DETECTION *****/

    double NOTE_BOUNDARY = LENGTH / 2 + Units.inchesToMeters(Settings.LENGTH / 2);

    /**** EMPTY FIELD POSES ****/

    Pose2d EMPTY_FIELD_POSE2D = new Pose2d(new Translation2d(-1, -1), new Rotation2d());
    Pose3d EMPTY_FIELD_POSE3D = new Pose3d(-1, -1, 0, new Rotation3d());

    public static void clearFieldObject(FieldObject2d fieldObject)  {
        fieldObject.setPose(EMPTY_FIELD_POSE2D);
    }
}
