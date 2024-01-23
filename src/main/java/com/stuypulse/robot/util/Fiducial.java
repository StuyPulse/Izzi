package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Pose3d;

public class Fiducial {

    private final int fid;
    private final Pose3d location;

    public Fiducial(int fid, Pose3d location) {
        this.fid = fid;
        this.location = location;
    }

    public int getFID(){
        return fid;
    }

    public Pose3d getLocation() {
        return location;
    }
}
