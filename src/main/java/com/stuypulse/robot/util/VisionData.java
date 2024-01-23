package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionData {
    
    private final Pose3d outputPose;
    private final int[] fids;
    private final Pose3d cameraLocation;
    private final double timestamp;

    public VisionData(Pose3d outputPose, int[] fids, Pose3d cameraLocation, double timestamp) {
        this.outputPose = outputPose;
        this.fids = fids;
        this.cameraLocation = cameraLocation;
        this.timestamp = timestamp;
    }
    
    public Pose3d getPose() {
        return outputPose;
    }

    public int[] getFids() {
        return fids;
    }

    public Pose3d getCameraLocation() {
        return cameraLocation;
    }

    public double getTimestamp() {
        return timestamp;
    }
    
    public double getDistance() {
        return outputPose.getTranslation().getDistance(Field.getFiducial(getPrimaryID()).getLocation().getTranslation());
    }
    
    public int getPrimaryID() {
        if (fids.length == 0) return -1;
        return fids[0];
    }
}