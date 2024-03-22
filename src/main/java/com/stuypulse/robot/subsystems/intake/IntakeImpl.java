/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Motors.StatusFrame;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

public class IntakeImpl extends Intake {

    private final CANSparkFlex motor;
    private final DigitalInput sensor;

    private final BStream triggered;
    private final BStream stalling;

    private final PowerDistribution powerDistribution;

    protected IntakeImpl() {
        motor = new CANSparkFlex(Ports.Intake.MOTOR, MotorType.kBrushless);
        sensor = new DigitalInput(Ports.Intake.IR_SENSOR);

        triggered = BStream.create(sensor).not()
            .filtered(new BDebounce.Rising(Settings.Intake.Detection.TRIGGER_TIME));

        stalling = BStream.create(this::isMomentarilyStalling)
            .filtered(new BDebounceRC.Rising(Settings.Intake.Detection.STALL_TIME));
        
        Motors.disableStatusFrames(motor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_POSIITION, StatusFrame.ABS_ENCODER_VELOCITY);

        Motors.Intake.MOTOR_CONFIG.configure(motor);

        powerDistribution = new PowerDistribution();
    }

    @Override
    public void acquire() {
        motor.set(+Settings.Intake.ACQUIRE_SPEED);
    }

    @Override
    public void deacquire() {
        motor.set(-Settings.Intake.DEACQUIRE_SPEED);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        motor.setIdleMode(mode);
        motor.burnFlash();
    }

    // Detection

    private boolean isMomentarilyStalling() {
        return motor.getOutputCurrent() > Settings.Intake.Detection.STALL_CURRENT;
    }

    private boolean isStalling() {
        return stalling.get();
    }

    private boolean isTriggered() {
        return triggered.get();
    }

    // Not using stall detection, but keeping it as an option
    @Override
    public boolean hasNote() {
        return isTriggered();
    }

    @Override
    public boolean hasNotePartially() {
        return hasNote() || isStalling();
    }

    @Override
    public double getIntakeRollerSpeed() {
        return motor.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Intake/Speed", motor.get());
        SmartDashboard.putNumber("Intake/Current", motor.getOutputCurrent());

        SmartDashboard.putBoolean("Intake/Is Stalling", isStalling());
        SmartDashboard.putBoolean("Intake/Above Current Limit", isMomentarilyStalling());
        SmartDashboard.putBoolean("Intake/Has Note", isTriggered());
        SmartDashboard.putBoolean("Intake/Has Note (Raw)", !sensor.get());

        SmartDashboard.putNumber("Intake/Total Power", powerDistribution.getTotalPower());
        SmartDashboard.putNumber("Intake/Total Current", powerDistribution.getTotalCurrent());
        SmartDashboard.putNumber("Intake/Input Voltage", powerDistribution.getVoltage());
    }
}
