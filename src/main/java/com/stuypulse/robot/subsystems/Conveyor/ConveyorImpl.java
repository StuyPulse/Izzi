package com.stuypulse.robot.subsystems.Conveyor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;

import static com.stuypulse.robot.constants.Ports.Conveyor.*;
import static com.stuypulse.robot.constants.Settings.Conveyor.*;
import static com.stuypulse.robot.constants.Motors.Conveyor.*;


public class ConveyorImpl extends Conveyor {
    
    private final CANSparkMax gandalfMotor;
    private final CANSparkMax shooterFeederMotor;
    private final DigitalInput IRSensor;

    private BStream isAtShooter;

    protected ConveyorImpl(){
        gandalfMotor = new CANSparkMax(CONVEYOR_MOTOR_PORT, MotorType.kBrushless);
        shooterFeederMotor = new CANSparkMax(SHOOTER_FEEDER_MOTOR_PORT, MotorType.kBrushless);

        IRSensor = new DigitalInput(IR_SENSOR_PORT);

        GANDALF_MOTOR.configure(gandalfMotor);
        SHOOTER_FEEDER_MOTOR.configure(shooterFeederMotor);

        isAtShooter = 
            BStream.create(this::isLoaded)
                .filtered(new BDebounceRC.Rising(DEBOUNCE_TIME));
    }


    public boolean isLoaded() {
        return !IRSensor.get();
    }

    @Override
    public void toShooter() {
        gandalfMotor.set(GANDALF_SHOOTER_SPEED.get());
        shooterFeederMotor.set(SHOOTER_FEEDER_SPEED.get());
    }

    @Override
    public void toAmp() {
        gandalfMotor.set(GANDALF_AMP_SPEED.get());

    }

    public void stop() {
        gandalfMotor.set(0);
        shooterFeederMotor.set(0);
    }

    @Override
    public void periodic() {
        

    }

}