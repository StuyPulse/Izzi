package com.stuypulse.robot.subsystems.conveyor;

import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.util.ConveyorFunction;

public enum ConveyorMode {

    STOP((Intake intake, Conveyor conveyor, Amper amper) -> {
        intake.stop();
        conveyor.stop();
        amper.stopRoller();

        return false;
    }),
    
    SHOOT((Intake intake, Conveyor conveyor, Amper amper) -> {
        conveyor.toShooter();
        intake.acquire();

        // return Shooter.getInstance().noteShot();
        return false;
    }),
    
    SCORE_AMP((Intake intake, Conveyor conveyor, Amper amper) -> {
        amper.amp();

        return false;
    }),
    
    SCORE_TRAP((Intake intake, Conveyor conveyor, Amper amper) -> {
        amper.trap();

        return false;
    }),

        
    REVERSE_TRAP((Intake intake, Conveyor conveyor, Amper amper) -> {
        amper.reverseTrap();

        return false;
    }),
    
    TO_AMP((Intake intake, Conveyor conveyor, Amper amper) -> {
        if (Shooter.getInstance().atTargetSpeeds(300)) {
            conveyor.toAmp();
            intake.acquire();
            amper.fromConveyor();
        }

        return amper.hasNote();
    }),
    
    AMP_REVERSE((Intake intake, Conveyor conveyor, Amper amper) -> {
        amper.toConveyor();

        return false;
    }),
    
    GANDALF_AMP((Intake intake, Conveyor conveyor, Amper amper) -> {
        conveyor.toAmp();

        return false;
    }),

        
    GANDALF_SHOOT((Intake intake, Conveyor conveyor, Amper amper) -> {
        conveyor.toShooter();

        return false;
    }),
    
    INTAKE((Intake intake, Conveyor conveyor, Amper amper) -> {
        intake.acquire();

        return intake.hasNote();
    }),
    
    OUTTAKE((Intake intake, Conveyor conveyor, Amper amper) -> {
        intake.deacquire();

        return false;
    });

    private final ConveyorFunction<Intake, Conveyor, Amper> method;

    private final Intake intake;
    private final Conveyor conveyor;
    private final Amper amper;

    private ConveyorMode(ConveyorFunction<Intake, Conveyor, Amper> method) {
        this.method = method;

        intake = Intake.getInstance();
        conveyor = Conveyor.getInstance();
        amper = Amper.getInstance();
    }

    public void run() {
        method.accept(intake, conveyor, amper);
    }

    public boolean shouldStop() {
        return method.accept(intake, conveyor, amper);
    }

}
