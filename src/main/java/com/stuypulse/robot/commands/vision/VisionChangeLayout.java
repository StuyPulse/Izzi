package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.TheiaTagVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionChangeLayout extends InstantCommand {

    private final int[] ids;
    
    public VisionChangeLayout(int...ids) {
        this.ids = ids;
    }

    @Override
    public void initialize() {
        TheiaTagVision.getInstance().setTagWhitelist(ids);
    }
}
