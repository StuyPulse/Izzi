package com.stuypulse.robot.subsystems.vision;

import java.util.ArrayList;

import com.stuypulse.robot.util.vision.VisionData;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AprilTagVision extends SubsystemBase {
    
    private static final AprilTagVision instance;

    static {
        instance = new TheiaTagVision();
    }

    public static AprilTagVision getInstance() {
        return instance;
    }

    public abstract ArrayList<VisionData> getOutputs();

    public abstract void setTagWhitelist(int... fids);

}
