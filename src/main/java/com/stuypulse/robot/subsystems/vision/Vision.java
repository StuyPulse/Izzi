package com.stuypulse.robot.subsystems.vision;

import java.util.ArrayList;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.util.AprilTagCamera;
import com.stuypulse.robot.util.VisionData;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private static final Vision instance;

    static {
        instance = new Vision();
    }

    public static Vision getInstance() {
        return instance;
    }

    private final AprilTagCamera[] cameras;
    private final ArrayList<VisionData> outputs;

    public Vision() {
        this.cameras = new AprilTagCamera[Cameras.APRILTAG_CAMERAS.length];
        for (int i = 0; i < Cameras.APRILTAG_CAMERAS.length; i++) {
            cameras[i] = new AprilTagCamera(Cameras.APRILTAG_CAMERAS[i]);
        }
    }

    public ArrayList<VisionData> getOutputs() {
        return outputs;
    }

    public AprilTagCamera[] getCameras() {
        return cameras;
    }

    public void setFiducialLayout(int... fids) {
        for (AprilTagCamera camera : cameras)
            camera.setFiducialLayout(fids);
    }

    @Override
    public void periodic() {
        outputs.clear();

        for (AprilTagCamera camera : cameras) {
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

        SmartDashboard.putNumber(prefix + "/Distance to Tag", data.getDistance());

        SmartDashboard.putNumber(prefix + "/Pose Rotation", Units.radiansToDegrees(data.getPose().getRotation().getAngle()));
        SmartDashboard.putNumber(prefix + "/Timestamp", data.getTimestamp());
        SmartDashboard.putBoolean("Vision/Has Any Data", !outputs.isEmpty());
    }
}
 