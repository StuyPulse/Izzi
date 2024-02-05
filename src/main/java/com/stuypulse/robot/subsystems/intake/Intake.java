package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.util.IntakeVisualizer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {

    private static final Intake instance;
    private IntakeVisualizer intakeVisualizer;

    static {
        if (Robot.isReal()) {
            instance = new IntakeImpl();
        } else {
            instance = new IntakeSim();
        }
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

    public abstract boolean isIRTriggered();
    public abstract boolean hasNote();

    public abstract double getIntakeRollerSpeed();
    
    @Override
    public void periodic() {
        intakeVisualizer.update(isIRTriggered(), Conveyor.getInstance().isNoteAtShooter(), Amper.getInstance().hasNote(), getIntakeRollerSpeed(), Conveyor.getInstance().getGandalfMotorSpeed(), Conveyor.getInstance().getShooterFeederSpeed());
    }
}
