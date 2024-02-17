/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.climber;

import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings.RobotType;
import com.stuypulse.robot.util.ClimberVisualizer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Climber extends SubsystemBase {

    private static final Climber instance;

    static {
        if (Robot.ROBOT == RobotType.IZZI) {
            instance = new ClimberImpl();
        } else {
            instance = new ClimberSim();
        }
    }

    public static Climber getInstance() {
        return instance;
    }

    private final SmartNumber targetHeight;

    private final ClimberVisualizer climberVisualizer;

    public Climber() {
        targetHeight = new SmartNumber("Climber/Target Height", 0.0);
        climberVisualizer = new ClimberVisualizer();
    }

    public void setTargetHeight(double height) {
        targetHeight.set(height);
    }

    public final double getTargetHeight() {
        return targetHeight.get();
    }

    public final boolean isAtTargetHeight(double epsilonMeters) {
        return isAtLeftTargetHeight(epsilonMeters) && isAtRightTargetHeight(epsilonMeters);
    }

    public final boolean isAtLeftTargetHeight(double epsilonMeters) {
        return Math.abs(getTargetHeight() - getLeftHeight()) < epsilonMeters;
    }

    public final boolean isAtRightTargetHeight(double epsilonMeters) {
        return Math.abs(getTargetHeight() - getRightHeight()) < epsilonMeters;
    }

    public abstract double getHeight();

    public abstract double getLeftHeight();

    public abstract double getRightHeight();

    public abstract double getVelocity();

    /*** LIMITS ***/

    public abstract boolean atTop();

    public abstract boolean atBottom();

    public abstract void setVoltageOverride(double voltage);

    @Override
    public void periodic() {
        climberVisualizer.setLeftHeight(getLeftHeight());
        climberVisualizer.setRightHeight(getRightHeight());
    }
}
