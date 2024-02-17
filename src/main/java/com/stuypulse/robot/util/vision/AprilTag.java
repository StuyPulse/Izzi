/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.util.vision;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose3d;

/** This class stores information about a tag. */
public class AprilTag {

    private final int id;
    private final Pose3d blueLocation;

    public AprilTag(int id, Pose3d blueLocation) {
        this.id = id;
        this.blueLocation = blueLocation;
    }

    /**
     * Returns the ID of the tag.
     *
     * @return the ID of the tag
     */
    public int getID() {
        return id;
    }

    /**
     * Returns the location of the tag on the field.
     *
     * @return the location of the tag on the field
     */
    public Pose3d getLocation() {
        if (Robot.isBlue()) {
            return blueLocation;
        } else {
            return Field.transformToOppositeAlliance(blueLocation);
        }
    }
}
