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
import com.stuypulse.robot.constants.Settings.Shooter.Feedforward;
import com.stuypulse.robot.constants.Settings.Shooter.PID;
import com.stuypulse.robot.util.StupidFilter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

public class ShooterImpl extends Shooter {

    private final CANSparkFlex leftMotor;
    private final CANSparkFlex rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final Controller leftController;
    private final Controller rightController;

    private final StupidFilter leftVel;
    private final StupidFilter rightVel;

    protected ShooterImpl() {
        leftMotor = new CANSparkFlex(Ports.Shooter.LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new CANSparkFlex(Ports.Shooter.RIGHT_MOTOR, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftEncoder.setVelocityConversionFactor(1.0);
        rightEncoder.setVelocityConversionFactor(1.0);

        leftController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
            .add(new PIDController(PID.kP, PID.kI, PID.kD));
        rightController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
            .add(new PIDController(PID.kP, PID.kI, PID.kD));
        
        leftVel = new StupidFilter();
        rightVel = new StupidFilter();

        Motors.Shooter.LEFT_SHOOTER.configure(leftMotor);
        Motors.Shooter.RIGHT_SHOOTER.configure(rightMotor);
    }

    @Override
    public void stop() {
        leftMotor.setVoltage(0);
        rightMotor.setVoltage(0);
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
    public void periodic() {
        super.periodic();

        leftController.update(getLeftTargetRPM(), getLeftShooterRPM());
        rightController.update(getRightTargetRPM(), getRightShooterRPM());

        leftMotor.setVoltage(leftController.getOutput());
        rightMotor.setVoltage(rightController.getOutput());

        SmartDashboard.putNumber("Shooter/Right RPM", getRightShooterRPM());
        SmartDashboard.putNumber("Shooter/Left RPM", getLeftShooterRPM());

        SmartDashboard.putNumber("Shooter/Left Voltage", leftMotor.getBusVoltage() * leftMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Right Voltage", rightMotor.getBusVoltage() * rightMotor.getAppliedOutput());
    }
}
