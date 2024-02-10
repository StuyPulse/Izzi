package com.stuypulse.robot.util.vision;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * This class stores information about a tag.
 */
public class AprilTag {

    private final int id;
    private final Pose3d location;

    public AprilTag(int id, Pose3d location) {
        this.id = id;
        this.location = location;
    }

    /**
     * Returns the ID of the tag.
     * @return the ID of the tag
     */
    public int getID() {
        return id;
    }

    /**
     * Returns the location of the tag on the field.
     * @return the location of the tag on the field
     */
    public Pose3d getLocation() {
        return location;
    }
}
