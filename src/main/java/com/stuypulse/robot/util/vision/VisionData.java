/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.util.vision;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose3d;

/** This class stores pieces of data from the vision system. */
public class VisionData {

    private final Pose3d outputPose;
    private final int[] ids;
    private final double timestamp;
    private final double area;

    public VisionData(Pose3d outputPose, int[] ids, double timestamp, double area) {
        this.outputPose = outputPose;
        this.ids = ids;
        this.timestamp = timestamp;
        this.area = area;
    }

    /**
     * Returns the pose of the robot relative to the field.
     *
     * @return the pose of the robot relative to the field
     */
    public Pose3d getPose() {
        return outputPose;
    }

    /**
     * Returns the IDs of the tags detected.
     *
     * @return the IDs of the tags detected
     */
    public int[] getIDs() {
        return ids;
    }

    /**
     * Returns the timestamp of the vision data.
     *
     * @return the timestamp of the vision data
     */
    public double getTimestamp() {
        return timestamp;
    }

    /**
     * Returns the distance to the primary tag in each vision data entry.
     *
     * @param id the tag ID
     * @return the distance to the primary tag
     */
    public double getDistanceToPrimaryTag() {
        return outputPose
            .getTranslation()
            .getDistance(Field.getTag(getPrimaryID()).getLocation().getTranslation());
    }

    /**
     * Returns the primary tag ID (first id).
     *
     * @return the primary tag ID
     */
    public int getPrimaryID() {
        return ids.length == 0 ? -1 : ids[0];
    }

    /**
     * Returns the area percentage of the primary tag.
     *
     * @return the area percentage of the primary tag
     */
    public double getArea() {
        return area;
    }

    /**
     * Returns if the data is valid.
     *
     * @return if valid data
     */
    public boolean isValidData() {
        for (long id : ids) {
            boolean found = false;
            for (AprilTag tag : Field.APRILTAGS) {
                if (tag.getID() == id) {
                    found = true;
                    break;
                }
            }

            if (!found) {
                return false;
            }
        }

        if (Double.isNaN(outputPose.getX())
                || outputPose.getX() < 0
                || outputPose.getX() > Field.LENGTH)
            return false;
        if (Double.isNaN(outputPose.getY())
                || outputPose.getY() < 0
                || outputPose.getY() > Field.WIDTH)
            return false;
        if (Double.isNaN(outputPose.getZ())
                || outputPose.getZ() < -1
                || outputPose.getZ() > 1)
            return false;

        return true;
    }
}
