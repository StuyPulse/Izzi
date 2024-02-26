package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionDisable extends InstantCommand {

    @Override
    public void initialize() {
        Odometry.getInstance().setVisionEnabled(false);
    }
}
