package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSim extends Intake {

    private double motor;

    private SmartBoolean intakeIR;

    public IntakeSim() {
        motor = 0;
        intakeIR = new SmartBoolean("Intake/Sim IR Value", false);
    }

    @Override
    public void acquire() {
        motor = +Settings.Intake.ACQUIRE_SPEED.get();
    }

    @Override 
    public void deacquire() {
        motor = -Settings.Intake.DEACQUIRE_SPEED.get();
    }

    @Override
    public void stop() {
        motor = 0;
    }

    @Override
    public double getIntakeRollerSpeed() {
        return motor;
    }

    @Override
    public boolean hasNote() {
        return intakeIR.get();
    }

    @Override
    public void periodic() {
        super.periodic();
        
        SmartDashboard.putNumber("Intake/Speed", motor);
    }
}
