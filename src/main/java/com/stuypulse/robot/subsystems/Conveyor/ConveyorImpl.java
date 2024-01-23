package com.stuypulse.robot.subsystems.Conveyor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static com.stuypulse.robot.constants.Ports.Conveyor.*;
import static com.stuypulse.robot.constants.Settings.Conveyor.*;
import static com.stuypulse.robot.constants.Motors.Conveyor.*;
public class ConveyorImpl extends Conveyor {
    
    private final CANSparkMax gandalfMotor;
    private final CANSparkMax shooterFeederMotor;

    protected ConveyorImpl(){
        gandalfMotor = new CANSparkMax(CONVEYOR_MOTOR_PORT, MotorType.kBrushless);
        shooterFeederMotor = new CANSparkMax(SHOOTER_FEEDER_MOTOR_PORT, MotorType.kBrushless);

        GANDALF_MOTOR.configure(gandalfMotor);
        SHOOTER_FEEDER_MOTOR.configure(shooterFeederMotor);
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

    @Override
    public void periodic() {
        

    }

}