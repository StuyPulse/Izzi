package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.constants.Settings.Shooter.Feedforward;
import com.stuypulse.robot.constants.Settings.Shooter.PID;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.control.feedback.PIDController;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {

    private static final Shooter instance;
    
    static {
        if (RobotBase.isReal()) {
            instance = new ShooterImpl();
        } else {
            instance = new SimShooter();
        }
    }
    
    public static Shooter getInstance() {
        return instance;
    }

    private final SmartNumber leftTargetRPM;
    private final SmartNumber rightTargetRPM;

    private final Controller leftController;
    private final Controller rightController;
    
    public Shooter() {
        leftTargetRPM = new SmartNumber("Shooter/leftTargetRPM", 0);
        rightTargetRPM = new SmartNumber("Shooter/rightTargetRPM", 0);
        leftController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
                        .add(new PIDController(PID.kP, PID.kI, PID.kD));
        rightController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
                        .add(new PIDController(PID.kP, PID.kI, PID.kD));  
    }    

    public double getLeftTargetRPM() {
        return leftTargetRPM.get();
    }   

    public double getRightTargetRPM() {
        return rightTargetRPM.get();
    } 
    
    public void setLeftTargetRPM(Number leftTargetRPM) {
        this.leftTargetRPM.set(leftTargetRPM);
    }
    
    public void setRightTargetRPM(Number rightTargetRPM) {
        this.rightTargetRPM.set(rightTargetRPM);
    }

    public abstract void stop();
    public abstract double getLeftShooterRPM();
    public abstract double getRightShooterRPM();

    protected abstract void setLeftMotorVoltageImpl(double voltage);
    protected abstract void setRightMotorVoltageImpl(double voltage);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Left Target RPM",getLeftTargetRPM());
        SmartDashboard.putNumber("Shooter/Right Target RPM", getRightTargetRPM());
        
        leftController.update(getLeftTargetRPM(), getLeftShooterRPM());
        rightController.update(getRightTargetRPM(), getRightShooterRPM());
        setLeftMotorVoltageImpl(leftController.getOutput());
        setRightMotorVoltageImpl(rightController.getOutput());

        periodicallyCalled();
    }

    public void periodicallyCalled() {}
}