/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings.Feeder;
import com.stuypulse.robot.constants.Settings.Shooter.Feedforward;
import com.stuypulse.robot.constants.Settings.Shooter.PID;
import com.stuypulse.robot.util.StupidFilter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

public class ShooterImpl extends Shooter {

    private final CANSparkFlex leftMotor;
    private final CANSparkFlex rightMotor;
    private final CANSparkMax feederMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder feederEncoder;

    private final Controller leftController;
    private final Controller rightController;
    private final Controller feederController;

    private final StupidFilter leftVel;
    private final StupidFilter rightVel;

    protected ShooterImpl() {
        leftMotor = new CANSparkFlex(Ports.Shooter.LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new CANSparkFlex(Ports.Shooter.RIGHT_MOTOR, MotorType.kBrushless);
        feederMotor = new CANSparkMax(Ports.Conveyor.FEEDER, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        feederEncoder = feederMotor.getEncoder();

        leftEncoder.setVelocityConversionFactor(1.0);
        rightEncoder.setVelocityConversionFactor(1.0);
        feederEncoder.setVelocityConversionFactor(Feeder.GEARING);

        leftController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
            .add(new PIDController(PID.kP, PID.kI, PID.kD));
        rightController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
            .add(new PIDController(PID.kP, PID.kI, PID.kD));
        feederController = new MotorFeedforward(Feeder.Feedforward.kS, Feeder.Feedforward.kV, Feeder.Feedforward.kA).velocity()
            .add(new PIDController(Feeder.PID.kP, Feeder.PID.kI, Feeder.PID.kD));
        
        leftVel = new StupidFilter();
        rightVel = new StupidFilter();

        Motors.Shooter.LEFT_SHOOTER.configure(leftMotor);
        Motors.Shooter.RIGHT_SHOOTER.configure(rightMotor);
        Motors.Conveyor.SHOOTER_FEEDER_MOTOR.configure(feederMotor);
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
        feederMotor.stopMotor();
    }

    @Override
    public double getLeftShooterRPM() {
        return leftVel.get(leftEncoder.getVelocity());
    }

    @Override
    public double getRightShooterRPM() {
        return rightVel.get(rightEncoder.getVelocity());
    }

    @Override
    public double getFeederRPM() {
        return feederEncoder.getVelocity();
    }

    @Override
    public void periodic() {
        super.periodic();

        leftController.update(getLeftTargetRPM(), getLeftShooterRPM());
        rightController.update(getRightTargetRPM(), getRightShooterRPM());
        feederController.update(getFeederTargetRPM(), getFeederRPM());

        if (getLeftTargetRPM() == 0 && getRightTargetRPM() == 0 && getFeederTargetRPM() == 0) {
            stop();
        } else {
            leftMotor.setVoltage(leftController.getOutput());
            rightMotor.setVoltage(rightController.getOutput());
            feederMotor.setVoltage(feederController.getOutput());
        }

        SmartDashboard.putNumber("Shooter/Right RPM", getRightShooterRPM());
        SmartDashboard.putNumber("Shooter/Left RPM", getLeftShooterRPM());
        SmartDashboard.putNumber("Shooter/Feeder RPM", getFeederRPM());
        SmartDashboard.putNumber("Shooter/Feeder Target RPM", getFeederTargetRPM());

        SmartDashboard.putNumber("Shooter/Left Voltage", leftMotor.getBusVoltage() * leftMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Right Voltage", rightMotor.getBusVoltage() * rightMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Feeder Voltage", feederMotor.getBusVoltage() * feederMotor.getAppliedOutput());

        SmartDashboard.putNumber("Shooter/Left Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Right Current", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Feeder Current", feederMotor.getOutputCurrent());
    }
}
