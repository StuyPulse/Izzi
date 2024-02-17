/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.TheiaTagVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionChangeWhiteList extends InstantCommand {

    private final int[] ids;

    public VisionChangeWhiteList(int... ids) {
        this.ids = ids;
    }

    @Override
    public void initialize() {
        TheiaTagVision.getInstance().setTagWhitelist(ids);
    }
}
