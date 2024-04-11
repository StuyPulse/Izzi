/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.util.vision;

import com.stuypulse.robot.constants.Cameras.CameraConfig;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

/**
 * This class handles interactions between the robot code and the Theia AprilTag system through the
 * NetworkTables.
 */
public class TheiaCamera {

    private final String name;
    private final Pose3d cameraLocation;
    
    // Default Values
    private final int camera_id = 0;
    private final int camera_resolution_width = 1600;
    private final int camera_resolution_height = 1200;
    private final int camera_auto_exposure = 1;
    private final int camera_exposure = 10;
    private final double camera_gain = 0.0;
    private final double camera_brightness = 0.0;

    private final int camera_pixel_count = camera_resolution_height * camera_resolution_width;

    // NetworkTables
    private final DoubleSubscriber latencySub;
    private final IntegerSubscriber fpsSub;
    private final DoubleArraySubscriber poseSub;
    private final IntegerArraySubscriber idSub;
    private final IntegerSubscriber counterSub;
    private final DoubleArraySubscriber areaSub;
    private final DoubleArraySubscriber pixelSub;

    private final DoubleArrayPublisher layoutPub;

    private final StructPublisher<Pose3d> posePub;


    private double rawLatency;
    private long rawFPS;
    private double[] rawPose;
    private long[] rawids;
    private long rawCounter;
    private double[] rawAreas;
    private double[] rawPixelCoords;
    private long lastCounter;

    private final SmartBoolean enabled;

    public TheiaCamera(String name, Pose3d cameraLocation, String ip, int port) {
        this.name = name;
        this.cameraLocation = cameraLocation;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(this.name);

        NetworkTable configTable = table.getSubTable("config");
        configTable.getIntegerTopic("camera_id").publish().set(camera_id);
        configTable.getIntegerTopic("camera_resolution_width").publish().set(camera_resolution_width);
        configTable.getIntegerTopic("camera_resolution_height").publish().set(camera_resolution_height);
        configTable.getIntegerTopic("camera_auto_exposure").publish().set(camera_auto_exposure);
        configTable.getIntegerTopic("camera_exposure").publish().set(camera_exposure);
        configTable.getDoubleTopic("camera_gain").publish().set(camera_gain);
        configTable.getDoubleTopic("camera_brightness").publish().set(camera_brightness);

        layoutPub = configTable.getDoubleArrayTopic("fiducial_layout").publish();
        layoutPub.set(Field.getLayoutAsDoubleArray(Field.APRILTAGS));

        NetworkTable outputTable = table.getSubTable("output");
        latencySub = outputTable.getDoubleTopic("latency").subscribe(0, PubSubOption.periodic(0.02));
        fpsSub = outputTable.getIntegerTopic("fps").subscribe(0, PubSubOption.periodic(0.02));
        poseSub = outputTable.getDoubleArrayTopic("pose").subscribe(new double[] {}, PubSubOption.periodic(0.02));
        idSub = outputTable.getIntegerArrayTopic("tids").subscribe(new long[] {}, PubSubOption.periodic(0.02));
        counterSub = outputTable.getIntegerTopic("update_counter").subscribe(0, PubSubOption.periodic(0.02));
        areaSub = outputTable.getDoubleArrayTopic("areas").subscribe(new double[] {}, PubSubOption.periodic(0.02));
        pixelSub = outputTable.getDoubleArrayTopic("pixel_coords").subscribe(new double[] {}, PubSubOption.periodic(0.02));

        posePub = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/Vision/" + getName() + "/Pose3d", Pose3d.struct).publish();

        enabled = new SmartBoolean(name + "/Enabled", true);
        
        PortForwarder.add(port, "10.6.94." + ip, 5802);
    }

    public TheiaCamera(CameraConfig config) {
        this(config.getName(), config.getLocation(), config.getIP(), config.getForwardedPort());
    }

    /**
     * Returns the name of the camera.
     *
     * @return the name of the camera
     */
    public String getName() {
        return name;
    }

    /**
     * Returns the FPS of the data that is coming from the camera.
     *
     * @return the FPS of the data that is coming from the camera
     */
    public int getFPS() {
        return (int) rawFPS;
    }

    private boolean hasData() {
        return rawPose.length > 0 &&
               rawids.length > 0 &&
               rawAreas.length > 0;
    }

    /** Pull the data from the NetworkTables and store it in the class. */
    private void updateData() {
        rawLatency = latencySub.get();
        rawFPS = (int) fpsSub.get();
        rawPose = poseSub.get();
        rawids = idSub.get();
        rawCounter = counterSub.get();
        rawAreas = areaSub.get();
        rawPixelCoords = pixelSub.get();
    }

    /**
     * Helper class that returns the rawPose as a Pose3d.
     *
     * @return the rawPose as a Pose3d
     */
    private Pose3d getDataAsPose3d() {
        return new Pose3d(
                new Translation3d(rawPose[0], rawPose[1], rawPose[2]),
                new Rotation3d(rawPose[3], rawPose[4], rawPose[5]));
    }

    /**
     * Returns the pose of the robot relative to the field.
     *
     * @return the pose of the robot relative to the field
     */
    private Pose3d getRobotPose() {
        return getDataAsPose3d()
            .transformBy(new Transform3d(cameraLocation.getTranslation(), cameraLocation.getRotation())
                .inverse());
    }

    /**
     * Returns the IDs of the tags detected.
     *
     * @return the IDs of the tags detected
     */
    private int[] getIDs() {
        int[] ids = new int[rawids.length];
        for (int i = 0; i < rawids.length; i++) {
            ids[i] = (int) rawids[i];
        }
        return ids;
    }

    public void setEnabled(boolean enabled) {
        this.enabled.set(enabled);
    }

    /**
     * Returns an Optional holding the vision data from the camera.
     *
     * @return the vision data from the camera in an Optional
     */
    public Optional<VisionData> getVisionData() {
        updateData();

        posePub.set(Field.EMPTY_FIELD_POSE3D);

        if (!hasData()) return Optional.empty();
        if (!enabled.get()) return Optional.empty();

        double fpgaTime = latencySub.getLastChange() / 1_000_000.0;
        double timestamp = fpgaTime - Units.millisecondsToSeconds(rawLatency);

        // if (rawCounter - lastCounter < 1) {
        //     lastCounter = rawCounter;

        //     return Optional.empty();
        // }

        lastCounter = rawCounter;

        VisionData data = new VisionData(getRobotPose(), getIDs(), timestamp, rawAreas[0] / camera_pixel_count);

        if (!data.isValidData()) return Optional.empty();

        posePub.set(data.getPose());

        if (rawPixelCoords.length == 2) {
            SmartDashboard.putNumber("Vision/" + getName() + "/Primary Tag X", rawPixelCoords[0]);
            SmartDashboard.putNumber("Vision/" + getName() + "/Primary Tag Y", rawPixelCoords[1]);
        }

        return Optional.of(data);
    }

    /**
     * Sets the tag layout of the camera.
     *
     * @param ids the tag IDs
     */
    public void setTagLayout(int... ids) {
        layoutPub.set(Field.getLayoutAsDoubleArray(Field.getApriltagLayout(ids)));
    }
}
