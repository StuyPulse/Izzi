package com.stuypulse.robot.subsystems.conveyor;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ConveyorSim extends Conveyor {

    private double gandalfMotorSpeed;
    private double feederMotorSpeed;

    public ConveyorSim() {
        gandalfMotorSpeed = 0;
        feederMotorSpeed = 0;
    }

    @Override
    public double getGandalfSpeed() {
        return gandalfMotorSpeed;
    }

    @Override
    public double getFeederSpeed() {
        return feederMotorSpeed;
    }

    @Override
    public boolean isNoteAtShooter() {
        return gandalfMotorSpeed > 0 && feederMotorSpeed > 0;
    }

    @Override
    public void toShooter() {
        gandalfMotorSpeed = +Settings.Conveyor.GANDALF_SHOOTER_SPEED.get();
        feederMotorSpeed =  +Settings.Conveyor.FEEDER_SHOOTER_SPEED.get();
    }

    @Override
    public void toAmp() {
        gandalfMotorSpeed = -Settings.Conveyor.GANDALF_AMP_SPEED.get();
        feederMotorSpeed =  +Settings.Conveyor.FEEDER_AMP_SPEED.get();

    }

    @Override
    public void stop() {
        gandalfMotorSpeed = 0;
        feederMotorSpeed = 0;
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Conveyor/Gandalf Motor Speed", gandalfMotorSpeed);
        SmartDashboard.putNumber("Conveyor/Shooter Feeder Motor Spped", feederMotorSpeed);
    }
}
