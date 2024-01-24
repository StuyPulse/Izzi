package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {

    private static final Intake instance;

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
