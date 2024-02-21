/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.climber;

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

    private final ClimberVisualizer climberVisualizer;

    public Climber() {
        climberVisualizer = new ClimberVisualizer();
    }

    public abstract void toTop();

    public abstract void toBottom();

    public abstract void stop();

    public abstract void setVoltageOverride(double voltage);

    /*** SENSORS ***/

    public final double getHeight() {
        return (getLeftHeight() + getRightHeight()) / 2.0;
    }

    public abstract double getLeftHeight();

    public abstract double getRightHeight();

    public abstract double getVelocity();

    @Override
    public void periodic() {
        climberVisualizer.setLeftHeight(getLeftHeight());
        climberVisualizer.setRightHeight(getRightHeight());
    }
}
