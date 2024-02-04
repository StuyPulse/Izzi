package com.stuypulse.robot.subsystems.climber;

import com.stuypulse.robot.util.ClimberVisualizer;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Climber extends SubsystemBase {

    private static final Climber instance;

    static {
        if (RobotBase.isReal()) {
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
        return Math.abs(getTargetHeight() - getHeight()) < epsilonMeters;
    }
    
    public abstract double getHeight();
    public abstract double getVelocity();

    public abstract boolean atTop();
    public abstract boolean atBottom();

    public abstract void setVoltageOverride(double voltage);

    @Override
    public void periodic() {
        climberVisualizer.setHeight(getHeight());
    }
}
