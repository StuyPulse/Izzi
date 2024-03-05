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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

public class IntakeImpl extends Intake {

    private final CANSparkFlex intakeMotor;
    private final CANSparkFlex conveyorMotor;
    private final DigitalInput sensor;

    private final BStream triggered;
    private final BStream stalling;

    protected IntakeImpl() {
        intakeMotor = new CANSparkFlex(Ports.Intake.INTAKE_MOTOR, MotorType.kBrushless);
        conveyorMotor = new CANSparkFlex(Ports.Intake.CONVEYOR_MOTOR, MotorType.kBrushless);
        sensor = new DigitalInput(Ports.Intake.IR_SENSOR);

        triggered = BStream.create(sensor).not()
                .filtered(new BDebounce.Rising(Settings.Intake.Detection.TRIGGER_TIME));

        stalling = BStream.create(this::isMomentarilyStalling)
            .filtered(new BDebounceRC.Rising(Settings.Intake.Detection.STALL_TIME));

        Motors.disableStatusFrames(intakeMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER,
                StatusFrame.ABS_ENCODER_POSIITION, StatusFrame.ABS_ENCODER_VELOCITY);

        Motors.disableStatusFrames(conveyorMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER,
                StatusFrame.ABS_ENCODER_POSIITION, StatusFrame.ABS_ENCODER_VELOCITY);

        Motors.Intake.INTAKE_CONFIG.configure(intakeMotor);
        Motors.Intake.CONVEYOR_CONFIG.configure(conveyorMotor);
    }

    @Override
    public void acquire() {
        intakeMotor.set(Settings.Intake.ACQUIRE_SPEED);
        conveyorMotor.set(Settings.Intake.ACQUIRE_SPEED);
    }

    @Override
    public void deacquire() {
        intakeMotor.set(-Settings.Intake.DEACQUIRE_SPEED);
        conveyorMotor.set(-Settings.Intake.DEACQUIRE_SPEED);
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
        conveyorMotor.stopMotor();
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        intakeMotor.setIdleMode(mode);
        conveyorMotor.setIdleMode(mode);
        intakeMotor.burnFlash();
        conveyorMotor.burnFlash();
    }

    // Detection

    private boolean isMomentarilyStalling() {
        return intakeMotor.getOutputCurrent() > Settings.Intake.Detection.STALL_CURRENT;
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
        return intakeMotor.get(); // both motors are at the same anyways // subject to change
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Intake/Intake Motor Speed", conveyorMotor.get());
        SmartDashboard.putNumber("Intake/Conveyor Motor Speed", conveyorMotor.get());
        SmartDashboard.putNumber("Intake/Current Intake Motor", intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Current Conveyor Motor", intakeMotor.getOutputCurrent());

        SmartDashboard.putBoolean("Intake/Is Stalling", isStalling());
        SmartDashboard.putBoolean("Intake/Above Current Limit", isMomentarilyStalling());
        SmartDashboard.putBoolean("Intake/Has Note", isTriggered());
    }
}
