package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.util.IntakeVisualizer;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {

    private static final Intake instance;
    private IntakeVisualizer intakeVisualizer;

    static {
        instance = new IntakeImpl();
    }

    public static Intake getInstance() {
        return instance;
    }

    public Intake() {
        intakeVisualizer = new IntakeVisualizer();
    }

    public abstract void acquire();
    public abstract void deacquire();
    public abstract void stop();

    public abstract boolean hasNote();

    SmartBoolean intakeIR = new SmartBoolean("TESTING/Intake IR", false);
    SmartBoolean shooterIR = new SmartBoolean("TESTING/Shooter IR", false);
    SmartBoolean amperIR = new SmartBoolean("TESTING/Amper IR", false);
    

    @Override
    public void periodic() {
        intakeVisualizer.update(intakeIR.get(), shooterIR.get(), amperIR.get(), true);
    }
    
}
