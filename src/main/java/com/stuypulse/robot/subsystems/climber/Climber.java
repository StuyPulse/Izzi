package com.stuypulse.robot.subsystems.climber;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Climber extends SubsystemBase {
    private static final Climber instance;
    
    private final SmartNumber targetHeight;
    protected final BangBangController controller;

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

    @Override
    public void periodic() {
        setVoltage(controller.calculate(getTargetHeight(), getHeight()));
        periodicallyCalled();
    }

    public void periodicallyCalled() {};
}
