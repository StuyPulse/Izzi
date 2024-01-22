package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// One CANSParkMax motor
// One IR Sensor

// things to do:
// acquire note
// deaquire note
// stop
// check if a note is inside

public abstract class Intake extends SubsystemBase {

    private static final Intake instance;
    
    // change this later when we make sims and stuff
    static {
        instance = new IntakeImpl();
    }

    public static Intake getInstance() {
        return instance;
    }

    public abstract void acquire();
    public abstract void deacquire();
    public abstract void stop();

    public abstract boolean hasNote();
    
}
