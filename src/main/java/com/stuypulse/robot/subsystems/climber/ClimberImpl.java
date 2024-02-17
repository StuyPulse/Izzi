/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.climber;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.Optional;

public class ClimberImpl extends Climber {

    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;

    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;

    private final DigitalInput topRightLimit;
    private final DigitalInput topLeftLimit;
    private final DigitalInput bottomRightLimit;
    private final DigitalInput bottomLeftLimit;

    private Optional<Double> voltageOverride;

    protected ClimberImpl() {
        rightMotor = new CANSparkMax(Ports.Climber.RIGHT_MOTOR, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Ports.Climber.LEFT_MOTOR, MotorType.kBrushless);

        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

        rightEncoder.setPositionConversionFactor(Settings.Climber.Encoder.POSITION_CONVERSION);
        leftEncoder.setPositionConversionFactor(Settings.Climber.Encoder.POSITION_CONVERSION);

        rightEncoder.setVelocityConversionFactor(Settings.Climber.Encoder.VELOCITY_CONVERSION);
        leftEncoder.setVelocityConversionFactor(Settings.Climber.Encoder.VELOCITY_CONVERSION);

        topRightLimit = new DigitalInput(Ports.Climber.TOP_RIGHT_LIMIT);
        topLeftLimit = new DigitalInput(Ports.Climber.TOP_LEFT_LIMIT);
        bottomRightLimit = new DigitalInput(Ports.Climber.BOTTOM_RIGHT_LIMIT);
        bottomLeftLimit = new DigitalInput(Ports.Climber.BOTTOM_LEFT_LIMIT);

        voltageOverride = Optional.empty();

        Motors.Climber.LEFT_MOTOR.configure(leftMotor);
        Motors.Climber.RIGHT_MOTOR.configure(rightMotor);
    }

    @Override
    public void setTargetHeight(double height) {
        super.setTargetHeight(height);

        voltageOverride = Optional.empty();
    }

    @Override
    public double getHeight() {
        return getLeftHeight() + getRightHeight() / 2;
    }

    @Override
    public double getLeftHeight() {
        return leftEncoder.getPosition();
    }

    @Override
    public double getRightHeight() {
        return rightEncoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
    }

    /*** LIMITS ***/

    @Override
    public boolean atTop() {
        return leftAtTop() || rightAtTop();
    }

    private boolean leftAtTop() {
        return !topLeftLimit.get();
    }

    private boolean rightAtTop() {
        return !topRightLimit.get();
    }

    @Override
    public boolean atBottom() {
        return leftAtBottom() || rightAtBottom();
    }

    private boolean leftAtBottom() {
        return !bottomRightLimit.get();
    }

    private boolean rightAtBottom() {
        return !bottomLeftLimit.get();
    }

    @Override
    public void setVoltageOverride(double voltage) {
        voltageOverride = Optional.of(voltage);
    }

    private void setVoltage(double voltage) {
        if (atTop() && voltage > 0) {
            DriverStation.reportWarning("Climber Top Limit Reached", false);
            voltage = 0.0;
        } else if (atBottom() && voltage < 0) {
            DriverStation.reportWarning("Climber Bottom Limit Reached", false);
            voltage = 0.0;
        }

        rightMotor.setVoltage(voltage);
        leftMotor.setVoltage(voltage);
    }

    private void setLeftVoltage(double voltage) {
        if (leftAtTop() && voltage > 0) {
            DriverStation.reportWarning("Climber Top Left Limit Reached", false);
            voltage = 0.0;
        } else if (leftAtBottom() && voltage < 0) {
            DriverStation.reportWarning("Climber Bottom Left Limit Reached", false);
            voltage = 0.0;
        }

        leftMotor.setVoltage(voltage);
    }

    private void setRightVoltage(double voltage) {
        if (rightAtTop() && voltage > 0) {
            DriverStation.reportWarning("Climber Top Right Limit Reached", false);
            voltage = 0.0;
        } else if (rightAtBottom() && voltage < 0) {
            DriverStation.reportWarning("Climber Bottom Right Limit Reached", false);
            voltage = 0.0;
        }

        rightMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (voltageOverride.isPresent()) {
            setVoltage(voltageOverride.get());
        } else {
            if (isAtLeftTargetHeight(Settings.Climber.BangBang.THRESHOLD)) {
                setLeftVoltage(0.0);
            } else if (getLeftHeight() > getTargetHeight()) {
                setLeftVoltage(-Settings.Climber.BangBang.CONTROLLER_VOLTAGE);
            } else {
                setLeftVoltage(+Settings.Climber.BangBang.CONTROLLER_VOLTAGE);
            }

            if (isAtRightTargetHeight(Settings.Climber.BangBang.THRESHOLD)) {
                setRightVoltage(0.0);
            } else if (getRightHeight() > getTargetHeight()) {
                setRightVoltage(-Settings.Climber.BangBang.CONTROLLER_VOLTAGE);
            } else {
                setRightVoltage(+Settings.Climber.BangBang.CONTROLLER_VOLTAGE);
            }
        }

        SmartDashboard.putNumber("Climber/Left Voltage", leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Climber/Right Voltage", rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());
        SmartDashboard.putNumber("Climber/Left Height", getLeftHeight());
        SmartDashboard.putNumber("Climber/Right Height", getRightHeight());
        SmartDashboard.putNumber("Climber/Velocity", getVelocity());

        if (atBottom()) {
            leftEncoder.setPosition(Settings.Climber.MIN_HEIGHT);
            rightEncoder.setPosition(Settings.Climber.MIN_HEIGHT);
        }
    }
}
