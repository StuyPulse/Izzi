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

    private final SmartNumber leftTargetHeight;
    private final SmartNumber rightTargetHeight;

    private final ClimberVisualizer climberVisualizer;

    public Climber() {
        leftTargetHeight = new SmartNumber("Climber/Left Target Height", 0.0);
        rightTargetHeight = new SmartNumber("Climber/Right Target Height", 0.0);

        climberVisualizer = new ClimberVisualizer();
    }

    public void setLeftTargetHeight(double leftHeight) {
        leftTargetHeight.set(leftHeight);
    }

    public void setRightTargetHeight(double rightHeight) {
        rightTargetHeight.set(rightHeight);
    }

    public final double getLeftTargetHeight() {
        return leftTargetHeight.get();
    }

    public final double getRightTargetHeight() {
        return leftTargetHeight.get();
    }

    public final boolean isAtTargetHeight(double epsilonMeters) {
        return isAtLeftTargetHeight(epsilonMeters) && isAtRightTargetHeight(epsilonMeters);
    }

    public final boolean isAtLeftTargetHeight(double epsilonMeters) {
        return Math.abs(getLeftTargetHeight() - getLeftHeight()) < epsilonMeters;
    }

    public final boolean isAtRightTargetHeight(double epsilonMeters) {
        return Math.abs(getRightTargetHeight() - getRightHeight()) < epsilonMeters;
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
