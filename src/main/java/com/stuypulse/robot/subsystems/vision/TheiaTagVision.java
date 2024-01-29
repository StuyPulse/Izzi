package com.stuypulse.robot.subsystems.vision;

import java.util.ArrayList;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.util.vision.TheiaCamera;
import com.stuypulse.robot.util.vision.VisionData;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TheiaTagVision extends AprilTagVision {

    private final TheiaCamera[] cameras;
    private final ArrayList<VisionData> outputs;

    protected TheiaTagVision() {
        this.cameras = new TheiaCamera[Cameras.APRILTAG_CAMERAS.length];
        for (int i = 0; i < Cameras.APRILTAG_CAMERAS.length; i++) {
            cameras[i] = new TheiaCamera(Cameras.APRILTAG_CAMERAS[i]);
        }
        
        outputs = new ArrayList<VisionData>();
    }

    /**
     * Returns the outputs of the vision system.
     * @return the outputs of the vision system
     */
    @Override
    public ArrayList<VisionData> getOutputs() {
        return outputs;
    }

    /**
     * Sets the fiducial layout of the vision system.
     * @param fids the fiducial IDs
     */
    @Override
    public void setTagWhitelist(int... fids) {
        for (TheiaCamera camera : cameras) {
            camera.setFiducialLayout(fids);
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        outputs.clear();

        for (TheiaCamera camera : cameras) {
            if (camera.getVisionData().isPresent()) {
                outputs.add(camera.getVisionData().get());
                updateTelemetry(getName(), camera.getVisionData().get());
            }
        }
    }

    private void updateTelemetry(String prefix, VisionData data) {
        SmartDashboard.putNumber(prefix + "/Pose X", data.getPose().getX());
        SmartDashboard.putNumber(prefix + "/Pose Y", data.getPose().getY());
        SmartDashboard.putNumber(prefix + "/Pose Z", data.getPose().getZ());

        SmartDashboard.putNumber(prefix + "/Distance to Tag", data.getDistanceToFiducial(data.getPrimaryID()));

        SmartDashboard.putNumber(prefix + "/Pose Rotation", Units.radiansToDegrees(data.getPose().getRotation().getAngle()));
        SmartDashboard.putNumber(prefix + "/Timestamp", data.getTimestamp());
        SmartDashboard.putBoolean("Vision/Has Any Data", !outputs.isEmpty());
    }
}
 