package com.stuypulse.robot.util.vision;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * This class stores information about a fiducial.
 */
public class Fiducial {

    private final int fid;
    private final Pose3d location;

    public Fiducial(int fid, Pose3d location) {
        this.fid = fid;
        this.location = location;
    }

    /**
     * Returns the ID of the fiducial.
     * @return the ID of the fiducial
     */
    public int getFID(){
        return fid;
    }

    /**
     * Returns the location of the fiducial on the field.
     * @return the location of the fiducial on the field
     */
    public Pose3d getLocation() {
        return location;
    }
}
