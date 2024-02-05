package com.stuypulse.robot.subsystems.conveyor;

import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * field:
 * - Motor (cansparkmax, kraken?)
 * - shooter IR sensor
 * - set mode
 * 
 * Methods
 * - toAmp()
 * - toShooter()
 */

public abstract class Conveyor extends SubsystemBase {

    private static final Conveyor instance;

    static{
        if (Robot.isReal()) {
            instance = new ConveyorImpl();
        } else {
            instance = new ConveyorSim();
        }
    }

    public static Conveyor getInstance(){
        return instance;
    };

    public abstract double getGandalfMotorSpeed();
    public abstract double getShooterFeederSpeed();

    public abstract void toShooter();
    public abstract void toAmp();
    public abstract void stop();

    public abstract boolean isNoteAtShooter();
}
    