package com.stuypulse.robot.subsystems.climber;

import java.util.Optional;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Climber extends SubsystemBase {
    private static final Climber instance;
    
    private final SmartNumber targetHeight;

    protected Optional<Double> driveVoltage;

    static {
        if (RobotBase.isReal()) {
            instance = new ClimberImpl();
        } else {
            instance = new SimClimber();
        }
    }

    public static Climber getInstance() {
        return instance;
    }

    public Climber() {
        targetHeight = new SmartNumber("Climber/Target Height", 0.0);
    }

    public void setTargetHeight(double height) {
        targetHeight.set(height);

        driveVoltage = Optional.empty();
    }

    public double getTargetHeight() {
        return targetHeight.get();
    }
    
    public abstract double getHeight();
    public abstract double getVelocity();

    public abstract boolean atTop();
    public abstract boolean atBottom();

    public void setVoltageOverride(double voltage) {
        driveVoltage = Optional.of(voltage);
    }
}
