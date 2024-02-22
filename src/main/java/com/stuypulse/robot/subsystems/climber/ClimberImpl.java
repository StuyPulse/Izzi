/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.climber;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;

import edu.wpi.first.wpilibj.DigitalInput;
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

    // private final DigitalInput topRightLimit;
    // private final DigitalInput topLeftLimit;
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

        // topRightLimit = new DigitalInput(Ports.Climber.TOP_RIGHT_LIMIT);
        // topLeftLimit = new DigitalInput(Ports.Climber.TOP_LEFT_LIMIT);
        bottomRightLimit = new DigitalInput(Ports.Climber.BOTTOM_RIGHT_LIMIT);
        bottomLeftLimit = new DigitalInput(Ports.Climber.BOTTOM_LEFT_LIMIT);

        voltageOverride = Optional.empty();

        Motors.Climber.LEFT_MOTOR.configure(leftMotor);
        Motors.Climber.RIGHT_MOTOR.configure(rightMotor);
    }

    @Override
    public void toTop() {
        rightMotor.setVoltage(+Settings.Climber.Control.UP_VOLTAGE);
        leftMotor.setVoltage(+Settings.Climber.Control.UP_VOLTAGE);
    }

    @Override
    public void toBottom() {
        rightMotor.setVoltage(-Settings.Climber.Control.DOWN_VOLTAGE);
        leftMotor.setVoltage(-Settings.Climber.Control.DOWN_VOLTAGE);
    }

    @Override
    public void stop() {
        rightMotor.setVoltage(0.0);
        leftMotor.setVoltage(0.0);
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

    // private boolean atTop() {
    //     return leftAtTop() || rightAtTop();
    // }

    // private boolean leftAtTop() {
    //     return !topLeftLimit.get();
    // }

    // private boolean rightAtTop() {
    //     return !topRightLimit.get();
    // }

    private boolean atBottom() {
        return leftAtBottom() || rightAtBottom();
    }

    private boolean leftAtBottom() {
        return !bottomRightLimit.get();
    }

    private boolean rightAtBottom() {
        return !bottomLeftLimit.get();
    }

    private boolean isLeftStalling() {
        return rightMotor.getOutputCurrent() > Settings.Climber.Control.STALL_CURRENT;
    }

    private boolean isRightStalling() {
        return leftMotor.getOutputCurrent() > Settings.Climber.Control.STALL_CURRENT;
    }

    @Override
    public void setVoltageOverride(double voltage) {
        voltageOverride = Optional.of(voltage);
    }

    private double getLeftVoltage() {
        return leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    }

    private double getRightVoltage() {
        return rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (voltageOverride.isPresent()) {
            leftMotor.setVoltage(voltageOverride.get());
            rightMotor.setVoltage(voltageOverride.get());
        } else {
            if (getLeftVoltage() > 0 && isLeftStalling()) leftMotor.stopMotor();
            if (getRightVoltage() > 0 && isRightStalling()) rightMotor.stopMotor();

            if (getLeftVoltage() < 0 && isLeftStalling()) leftMotor.setVoltage(-Settings.Climber.Control.CLIMB_VOLTAGE);
            if (getRightVoltage() < 0 && isRightStalling()) rightMotor.setVoltage(-Settings.Climber.Control.CLIMB_VOLTAGE);
        }

        SmartDashboard.putNumber("Climber/Left Voltage", getLeftVoltage());
        SmartDashboard.putNumber("Climber/Right Voltage", getRightVoltage());
        SmartDashboard.putNumber("Climber/Left Height", getLeftHeight());
        SmartDashboard.putNumber("Climber/Right Height", getRightHeight());
        SmartDashboard.putNumber("Climber/Left Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Climber/Right Current", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Climber/Velocity", getVelocity());

        SmartDashboard.putBoolean("Climber/Left At Bottom", leftAtBottom());
        SmartDashboard.putBoolean("Climber/Right At Bottom", rightAtBottom());

        // SmartDashboard.putBoolean("Climber/Left At Top", leftAtTop());
        // SmartDashboard.putBoolean("Climber/Right At Top", leftAtTop());

        SmartDashboard.putBoolean("Climber/Left Stalling", isLeftStalling());
        SmartDashboard.putBoolean("Climber/Right Stalling", isRightStalling());

        if (atBottom()) {
            leftEncoder.setPosition(Settings.Climber.MIN_HEIGHT);
            rightEncoder.setPosition(Settings.Climber.MIN_HEIGHT);
        }
    }
}
