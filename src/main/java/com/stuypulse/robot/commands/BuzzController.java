package com.stuypulse.robot.commands;

import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class BuzzController extends InstantCommand {
    Gamepad driver;
    double intensity;
    public BuzzController(Gamepad driver, double intensity) {
        this.driver = driver;
        this.intensity = intensity;
    }

    @Override
    public void initialize() {
        driver.setRumble(intensity);
    }
}
