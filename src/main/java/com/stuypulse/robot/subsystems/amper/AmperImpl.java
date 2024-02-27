/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.amper;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ElevatorFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Motors.StatusFrame;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.util.StupidFilter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.Optional;

public class AmperImpl extends Amper {

    private final CANSparkMax scoreMotor;
    private final CANSparkMax liftMotor;
    private final RelativeEncoder liftEncoder;
    private final RelativeEncoder scoreEncoder;

    // private final DigitalInput alignedSwitch;
    private final DigitalInput minSwitch;
    // private final DigitalInput maxSwitch;
    private final DigitalInput ampIRSensor;

    private final Controller controller;

    private Optional<Double> voltageOverride;

    private final SmartNumber maxVelocity;
    private final SmartNumber maxAcceleration;

    private final StupidFilter liftHeight;

    protected AmperImpl() {
        scoreMotor = new CANSparkMax(Ports.Amper.SCORE, MotorType.kBrushless);
        scoreEncoder = scoreMotor.getEncoder();
        liftMotor = new CANSparkMax(Ports.Amper.LIFT, MotorType.kBrushless);
        liftEncoder = liftMotor.getEncoder();

        scoreEncoder.setPositionConversionFactor(Settings.Amper.Score.SCORE_MOTOR_CONVERSION);

        liftEncoder.setPositionConversionFactor(Settings.Amper.Lift.Encoder.POSITION_CONVERSION);
        liftEncoder.setVelocityConversionFactor(Settings.Amper.Lift.Encoder.VELOCITY_CONVERSION);

        // alignedSwitch = new DigitalInput(Ports.Amper.ALIGNED_BUMP_SWITCH);
        minSwitch = new DigitalInput(Ports.Amper.LIFT_BOTTOM_LIMIT);
        // maxSwitch = new DigitalInput(Ports.Amper.LIFT_TOP_LIMIT);
        ampIRSensor = new DigitalInput(Ports.Amper.AMP_IR);

        maxVelocity = new SmartNumber("Amper/Lift/Max Velocity", Lift.VEL_LIMIT);
        maxAcceleration = new SmartNumber("Amper/Lift/Max Acceleration", Lift.ACCEL_LIMIT);

        controller = new MotorFeedforward(Lift.Feedforward.kS, Lift.Feedforward.kV, Lift.Feedforward.kA).position()
            .add(new ElevatorFeedforward(Lift.Feedforward.kG))
            .add(new PIDController(Lift.PID.kP, Lift.PID.kI, Lift.PID.kD))
            .setSetpointFilter(new MotionProfile(maxVelocity, maxAcceleration));

        voltageOverride = Optional.empty();

        liftHeight = new StupidFilter("Lift Height");

        Motors.disableStatusFrames(liftMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_POSIITION, StatusFrame.ABS_ENCODER_VELOCITY);
        Motors.disableStatusFrames(scoreMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_POSIITION, StatusFrame.ABS_ENCODER_VELOCITY);

        Motors.Amper.LIFT_MOTOR.configure(liftMotor);
        Motors.Amper.SCORE_MOTOR.configure(scoreMotor);
    }

    /*** LIFT CONTROL ***/

    @Override
    public boolean liftAtBottom() {
        return !minSwitch.get();
    }

    @Override
    public boolean liftAtTop() {
        return false;
        // return !maxSwitch.get();
    }

    @Override
    public double getLiftHeight() {
        return liftHeight.get(liftEncoder.getPosition());
    }

    @Override
    public void stopLift() {
        liftMotor.stopMotor();
    }

    @Override
    public void setTargetHeight(double height) {
        super.setTargetHeight(height);

        voltageOverride = Optional.empty();
    }

    /*** IR SENSOR ***/

    @Override
    public boolean hasNote() {
        return !ampIRSensor.get();
    }

    /*** SCORE ROLLERS ***/

    @Override
    public void score() {
        scoreMotor.set(Settings.Amper.Score.SCORE_SPEED);
    }

    @Override
    public void fromConveyor() {
        scoreMotor.set(Settings.Amper.Score.FROM_CONVEYOR_SPEED);
    }

    @Override
    public void toConveyor() {
        scoreMotor.set(-Settings.Amper.Score.TO_CONVEYOR_SPEED.get());
    }

    @Override
    public void stopRoller() {
        scoreMotor.stopMotor();
    }

    // @Override
    // public boolean touchingAmp() {
    //     return !alignedSwitch.get();
    // }

    /*** LIFT CONFIG ***/

    @Override
    public void setVoltageOverride(double voltage) {
        voltageOverride = Optional.of(voltage);
    }

    @Override
    public void setConstraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity.set(maxVelocity);
        this.maxAcceleration.set(maxAcceleration);
    }

    @Override
    public double getNoteDistance() {
        return scoreEncoder.getPosition();
    }

    @Override
    public void periodic() {
        super.periodic();

        controller.update(getTargetHeight(), getLiftHeight());

        double voltage = voltageOverride.orElse(controller.getOutput());

        if (liftAtBottom() && voltage < 0 || liftAtTop() && voltage > 0) {
            voltage = 0;
        }

        boolean closeToBottom = getTargetHeight() == Settings.Amper.Lift.MIN_HEIGHT && isAtTargetHeight(0.04);
        if (closeToBottom && voltage > 0) {
            voltage = 0;
        }

        liftMotor.setVoltage(voltage);

        SmartDashboard.putNumber("Amper/Voltage", voltage);
        SmartDashboard.putNumber("Amper/Intake Speed", scoreMotor.get());
        SmartDashboard.putNumber("Amper/Lift Speed", liftMotor.get());
        SmartDashboard.putNumber("Amper/Intake Current", scoreMotor.getOutputCurrent());
        SmartDashboard.putNumber("Amper/Lift Current", liftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Amper/Lift Height", getLiftHeight());

        SmartDashboard.putBoolean("Amper/Has Note", hasNote());
        SmartDashboard.putBoolean("Amper/At Bottom", liftAtBottom());
        SmartDashboard.putBoolean("Amper/Close To Bottom", closeToBottom);
        
        SmartDashboard.putBoolean("Amper/Under Stage", Field.robotUnderStage());
    }
}
