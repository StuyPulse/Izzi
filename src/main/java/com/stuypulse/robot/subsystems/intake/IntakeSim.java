package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartBoolean;

public class IntakeSim extends Intake {

    private double motor;

    private SmartBoolean IR;

    public IntakeSim() {
        motor = 0;
        IR = new SmartBoolean("Intake/Sim IR Value", false);
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
        return IR.get();
    }
}
