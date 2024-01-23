package com.stuypulse.robot.util;

import java.util.Optional;

import com.stuypulse.robot.constants.Field;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagCamera {
    
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

    // NetworkTables
    private final DoubleSubscriber latencySub;
    private final IntegerSubscriber fpsSub;
    private final DoubleArraySubscriber poseSub;
    private final IntegerArraySubscriber fidSub;
    private final IntegerSubscriber counterSub;
                     
    private final DoubleArrayPublisher layoutPub;

    private double rawLatency;
    private long rawFPS;
    private double[] rawPose;
    private long[] rawfids;
    private long rawCounter;
    
    public AprilTagCamera(String name, Pose3d cameraLocation) {
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
        layoutPub.set(Field.getLayoutAsDoubleArray());

        NetworkTable outputTable = table.getSubTable("output");
        latencySub = outputTable.getDoubleTopic("latency").subscribe(0);
        fpsSub = outputTable.getIntegerTopic("fps").subscribe(0);
        poseSub = outputTable.getDoubleArrayTopic("pose").subscribe(new double[] {});
        fidSub = outputTable.getIntegerArrayTopic("fid").subscribe(new long[] {});
        counterSub = outputTable.getIntegerTopic("counter").subscribe(0);
    }
        
    public String getName() {
        return name;
    }

    private void updateData() {
        rawLatency = latencySub.get();
        rawFPS = (int) fpsSub.get();
        rawPose = poseSub.get();
        rawfids = fidSub.get();
        rawCounter = counterSub.get();
    }

    private Pose3d getData() {
        return new Pose3d(
            new Translation3d(
                rawPose[0], 
                rawLatency, 
                camera_auto_exposure), 
            new Rotation3d(
                camera_brightness, 
                camera_auto_exposure, 
                rawCounter));
    }

    private int getFPS() {
        return (int) rawFPS;
    }

    private int[] getFIDs() {
        int[] fids = new int[rawfids.length];
        for (int i = 0; i < rawfids.length; i++) fids[i] = (int) rawfids[i];
        return fids;
    }

    public Optional<VisionData> getVisionData() {
        updateData();

        double fpgaTime = latencySub.getLastChange() / 1_000_000.0;
        double timestamp = fpgaTime - Units.millisecondsToSeconds(rawLatency);

        return Optional.of(new VisionData(getData(), getFIDs(), cameraLocation, timestamp));
    }
}
