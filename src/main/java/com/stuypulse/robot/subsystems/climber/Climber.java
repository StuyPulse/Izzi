/*
 * climbs chain in endgame
 * - 2 motors (12:1 gear ratio)
 * - 2 encoders
 * - 4 limit switches (bottom and top, two on each side)
 * - hard stop at bottom of the lift
 * 
 * functions:
 * - get current height of climber
 * - go to specific heights
 * - work with the gravity of the robot (has kG)
 */

package com.stuypulse.robot.subsystems.climber;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Climber extends SubsystemBase {
    public static final Climber instance;
    
    public final SmartNumber targetHeight;
    public final BangBangController controller;

    static {
        instance = new ClimberImpl();
    }

    public static Climber getInstance() {
        return instance;
    }

    public Climber() {
        targetHeight = new SmartNumber("Climber/Target Height", getTargetHeight());
        controller = new BangBangController();
    }

    public void setTargetHeight(double height) {
        targetHeight.set(height);
    }

    public double getTargetHeight() {
        return targetHeight.get();
    }
    
    public abstract double getHeight();

    public abstract double getVelocity();
    public abstract void setVoltage(double voltage);

    public abstract boolean atTop();
    public abstract boolean atBottom();

    public abstract void periodic();
}
