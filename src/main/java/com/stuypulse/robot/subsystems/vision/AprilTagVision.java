/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.util.vision.VisionData;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public abstract class AprilTagVision extends SubsystemBase {

    private static final AprilTagVision instance;

    static {
        instance = new TheiaTagVision();
    }

    public static AprilTagVision getInstance() {
        return instance;
    }

    public abstract ArrayList<VisionData> getOutputs();

    public abstract void setTagWhitelist(int... ids);

    public abstract void setCameraEnabled(String name, boolean enabled);
}
