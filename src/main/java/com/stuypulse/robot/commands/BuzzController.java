/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class BuzzController extends InstantCommand {

    private static final double DEFAULT_BUZZ_TIME = 0.2;

    private final Gamepad driver;
    private final double intensity;

    private final StopWatch timer;
    private final double seconds;

    public BuzzController(Gamepad driver, double intensity, double seconds) {
        this.driver = driver;
        this.intensity = intensity;
        this.seconds = seconds;

        timer = new StopWatch();
    }

    public BuzzController(Gamepad driver, double intensity) {
        this(driver, intensity, DEFAULT_BUZZ_TIME);
    }

    public BuzzController(Gamepad driver) {
        this(driver, Assist.BUZZ_INTENSITY);
    }

    @Override
    public void initialize() {
        driver.setRumble(intensity);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.getTime() >= seconds;
    }

    @Override
    public void end(boolean interrupted) {
        driver.setRumble(0);
    }
}
