package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeImpl extends Intake {

    private final CANSparkMax motor;
    private final DigitalInput irSensor;

    private final BStream triggered;
    private final BStream stalling;

    public IntakeImpl() {
        motor = new CANSparkMax(Ports.Intake.MOTOR, MotorType.kBrushless);
        irSensor = new DigitalInput(Ports.Intake.IR_SENSOR);

        Motors.Intake.MOTOR_CONFIG.configure(motor);

        triggered = BStream.create(irSensor).not()
            .filtered(new BDebounce.Rising(Settings.Intake.Detection.TRIGGER_TIME));

        stalling = BStream.create(this::isMomentarilyStalling)
            .filtered(new BDebounceRC.Rising(Settings.Intake.Detection.STALL_TIME));
    }

    @Override
    public void acquire() {
        motor.set(Settings.Intake.ACQUIRE_SPEED.getAsDouble());
    }

    @Override 
    public void deacquire() {
        motor.set(Settings.Intake.DEACQUIRE_SPEED.getAsDouble());
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public double getSpeed() {
        return motor.get();
    }

    // Detection

    @Override
    public boolean isIRTriggered() {
        return irSensor.get();
    }

    private boolean isMomentarilyStalling() {
        return motor.getOutputCurrent() > Settings.Intake.Detection.STALL_CURRENT.getAsDouble();
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
    public void periodic() {
        super.periodic();
        
        SmartDashboard.putNumber("Intake/Speed", getSpeed());
        SmartDashboard.putNumber("Intake/Current", motor.getOutputCurrent());

        SmartDashboard.putBoolean("Intake/Is Stalling", isStalling());
        SmartDashboard.putBoolean("Intake/IR is triggered", isTriggered());
    }
}
