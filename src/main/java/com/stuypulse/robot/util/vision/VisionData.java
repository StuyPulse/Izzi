package com.stuypulse.robot.util.vision;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * This class stores pieces of data from the vision system.
 */
public class VisionData {
    
    private final Pose3d outputPose;
    private final int[] fids;
    private final double timestamp;

    public VisionData(Pose3d outputPose, int[] fids, double timestamp) {
        this.outputPose = outputPose;
        this.fids = fids;
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
        return fids.length == 0 ? -1 : fids[0];
    }

    /**
     * Returns if the data is valid.
     * @return if valid data
     */
    public boolean isValidData() {
        for (long id : fids) {
            boolean found = false;
            for (Fiducial f : Field.FIDUCIALS) {
                if (f.getFID() == id) {
                    found = true;
                    break;
                }
            }

            if (!found) {
                return false;
            }
        }

        if (Double.isNaN(outputPose.getX()) || outputPose.getX() < 0  || outputPose.getX() > Field.LENGTH) return false;
        if (Double.isNaN(outputPose.getY()) || outputPose.getY() < 0  || outputPose.getY() > Field.WIDTH) return false;
        if (Double.isNaN(outputPose.getZ()) || outputPose.getZ() < -1 || outputPose.getZ() > 1) return false;

        return true;
    }
}