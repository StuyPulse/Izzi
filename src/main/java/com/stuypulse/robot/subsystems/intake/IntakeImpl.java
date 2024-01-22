package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Motors.Intake;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SensorUtil;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class IntakeImpl extends Intake {

    private CANSparkMax motor;
    private DigitalInput sensor;

    private BStream tripping;

    private boolean acquiring;

    public IntakeImpl() {
        motor = new CANSparkMax(Ports.Intake.MOTOR, MotorType.kBrushless);
        sensor = new DigitalInput(Ports.Intake.SENSOR);

        Motors.Intake.MOTOR_CONFIG.configure(motor);
    }

    private boolean isMomentarilyTripped() {
        return !sensor.get(); // check if IR sensor returns true when not blocked idk if this is right
    }

    private void acquire() {
        motor.set(1);
    }

    private void deacquire() {
        motor.set(-1);
    }

    private void stop() {
        motor.set(0);
    }
    
    
}
