package com.stuypulse.robot.subsystems.conveyor;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;

public class ConveyorSim extends Conveyor {

    SmartNumber gandalfMotor = new SmartNumber("Conveyor/Gandalf Speed", 0);
    SmartNumber shooterFeederMotor = new SmartNumber("Conveyor/Shooter Feeder Speed", 0);

    @Override
    public double getGandalfMotorSpeed() {
        return gandalfMotor.get();
    }

    @Override
    public double getShooterFeederSpeed() {
        return shooterFeederMotor.get();
    }

    @Override
    public boolean isNoteAtShooter() {
        return false;
    }

    @Override
    public void toShooter() {
        gandalfMotor.set(Settings.Conveyor.GANDALF_SHOOTER_SPEED.get());
        shooterFeederMotor.set(Settings.Conveyor.SHOOTER_FEEDER_SPEED.get());
    }

    @Override
    public void toAmp() {
        gandalfMotor.set(Settings.Conveyor.GANDALF_AMP_SPEED.get());

    }

    @Override
    public void stop() {
        gandalfMotor.set(0);
        // shooterFeederMotor.set(0);
    }
}
