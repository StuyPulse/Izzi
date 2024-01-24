package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * This class stores pieces of data from the vision system.
 */
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
    
    /**
     * Returns the pose of the robot relative to the field.
     * @return the pose of the robot relative to the field
     */
    public Pose3d getPose() {
        return outputPose;
    }

    /**
     * Returns the IDs of the fiducials detected.
     * @return the IDs of the fiducials detected
     */
    public int[] getFids() {
        return fids;
    }

    /**
     * Returns the offset of the camera from the center of the robot.
     * @return the offset of the camera from the center of the robot
     */
    public Pose3d getCameraLocation() {
        return cameraLocation;
    }

    /**
     * Returns the timestamp of the vision data.
     * @return the timestamp of the vision data
     */
    public double getTimestamp() {
        return timestamp;
    }
    
    /**
     * Returns the distance to any fiducial on the field.
     * @param fid the fiducial ID
     * @return the distance to the fiducial
     */
    public double getDistanceToFiducial(int fid) {
        return outputPose.getTranslation().getDistance(Field.getFiducial(fid).getLocation().getTranslation());
    }
    
    /**
     * Returns the primary fiducial ID (first fid).
     * @return the primary fiducial ID
     */
    public int getPrimaryID() {
        if (fids.length == 0) return -1;
        return fids[0];
    }
}