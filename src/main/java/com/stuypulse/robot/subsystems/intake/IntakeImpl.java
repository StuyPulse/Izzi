package com.stuypulse.robot.subsystems.intake;

import static com.stuypulse.robot.constants.Motors.Intake.*;
import static com.stuypulse.robot.constants.Ports.Intake.*;
import static com.stuypulse.robot.constants.Settings.Intake.*;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeImpl extends Intake {

    private CANSparkMax motor;
    private DigitalInput sensor;

    private BStream triggered;

    private boolean acquiring;

    public IntakeImpl() {
        motor = new CANSparkMax(MOTOR, MotorType.kBrushless);
        sensor = new DigitalInput(SENSOR);

        MOTOR_CONFIG.configure(motor);

        triggered = BStream.create(sensor).filtered(new BDebounce.Rising(TRIGGER_TIME));
    }

    @Override
    public void acquire() {

    }

    @Override 
    public void deacquire() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean hasNote() {

    }

    @Override
    public void periodic() {
        
    }
}
