package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class ConveyorScoreNote extends Command {

    private final Conveyor conveyor;
    private final Intake intake;
    private final Amper amper;

    private boolean noteAtShooter;
    private boolean noteAtAmp;

    public ConveyorScoreNote() {
        conveyor = Conveyor.getInstance();
        intake = Intake.getInstance();
        amper = Amper.getInstance();

        addRequirements(conveyor, amper);
    };

    @Override
    public void initialize() {
        noteAtShooter = conveyor.isNoteAtShooter();
        noteAtAmp = amper.hasNote();
    }

    @Override
    public void execute() {
        if (noteAtAmp) {
            amper.score();
        } else if (noteAtShooter) {
            conveyor.toShooter();
            intake.acquire();
        }
    }

    @Override
    public void end(boolean interrupted) {
        amper.stopRoller();
        conveyor.stop();
    }
}