package com.stuypulse.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterImpl extends Shooter {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;


    public ShooterImpl(){
        leftMotor = new CANSparkMax(Ports.Shooter.LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Ports.Shooter.RIGHT_MOTOR, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        Motors.Shooter.LEFT_SHOOTER.configure(leftMotor);
        Motors.Shooter.RIGHT_SHOOTER.configure(rightMotor);
    }

    @Override
    public void stop() {
        setLeftTargetRPM(0);
        setRightTargetRPM(0);
    }

    @Override
    public double getLeftShooterRPM() {
        return leftEncoder.getVelocity();
    }

    @Override
    public double getRightShooterRPM() {
        return rightEncoder.getVelocity();
    }
    
    @Override
    public void setLeftMotorVoltageImpl(double voltage){
        leftMotor.setVoltage(voltage);
    }
    
    @Override
    public void setRightMotorVoltageImpl(double voltage){
        rightMotor.setVoltage(voltage);
    }
    
    @Override
    public void periodicallyCalled() {
        SmartDashboard.putNumber("Shooter/Left Target RPM",getLeftTargetRPM());
        SmartDashboard.putNumber("Shooter/Right Target RPM", getRightTargetRPM());
        
        SmartDashboard.putNumber("Shooter/Right RPM", getRightShooterRPM());
        SmartDashboard.putNumber("Shooter/Left RPM", getLeftShooterRPM());

        SmartDashboard.putNumber("Shooter/Left Voltage", leftMotor.getBusVoltage());;
        SmartDashboard.putNumber("Shooter/Right Voltage", rightMotor.getBusVoltage());
    }
}                                

