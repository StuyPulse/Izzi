package com.stuypulse.robot.subsystems.Conveyor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static com.stuypulse.robot.constants.Ports.Conveyor.*;
import static com.stuypulse.robot.constants.Settings.Conveyor.*;

public class ConveyorImpl extends Conveyor {
    
    private final CANSparkMax gandalfMotor;
    private final CANSparkMax shooterFeederMotor;

    private Conveyor.Direction target;

    protected ConveyorImpl(){
        gandalfMotor = new CANSparkMax(CONVEYOR_MOTOR_PORT, MotorType.kBrushless);
        shooterFeederMotor = new CANSparkMax(SHOOTER_FEEDER_MOTOR_PORT, MotorType.kBrushless);
        
        target = Conveyor.Direction.NONE;
    }

    public Conveyor.Direction getTarget() {
        return target;
    }

    public void setTarget(Conveyor.Direction target) {
        this.target = target;
    }

    @Override
    public void gandalfToShooter() {
        gandalfMotor.set(GANDALF_SHOOTER_SPEED.get());
    }

    @Override
    public void gandalfToAmp() {
        gandalfMotor.set(GANDALF_AMP_SPEED.get());
    }

    @Override
    public void gandalfStop() {
        gandalfMotor.set(0);
    }

    @Override
    public void feederForward() {
        shooterFeederMotor.set(SHOOTER_FEEDER_SPEED.get());
    }

    @Override
    public void feederStop() {
        shooterFeederMotor.set(0);
    }

    @Override
    public void periodic() {
        

    }

}