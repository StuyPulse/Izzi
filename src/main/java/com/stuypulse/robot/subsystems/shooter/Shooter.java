package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {

    private static final Shooter instance;
    
    static {
        if (RobotBase.isReal()) {
            instance = new ShooterImpl();
        } else {
            instance = new ShooterSim();
        }
    }
    
    public static Shooter getInstance() {
        return instance;
    }

    private final SmartNumber leftTargetRPM;
    private final SmartNumber rightTargetRPM;

    public Shooter() {
        leftTargetRPM = new SmartNumber("Shooter/Left Target RPM", 0);
        rightTargetRPM = new SmartNumber("Shooter/Right Target RPM", 0);
    }    

    public final double getLeftTargetRPM() {
        return leftTargetRPM.get();
    }   

    public final double getRightTargetRPM() {
        return rightTargetRPM.get();
    } 
    
    public final void setLeftTargetRPM(Number leftTargetRPM) {
        this.leftTargetRPM.set(leftTargetRPM);
    }
    
    public final void setRightTargetRPM(Number rightTargetRPM) {
        this.rightTargetRPM.set(rightTargetRPM);
    }

    public abstract void stop();
    public abstract double getLeftShooterRPM();
    public abstract double getRightShooterRPM();
}