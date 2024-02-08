package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;

import edu.wpi.first.wpilibj2.command.Command;

    public class ConveyorScoreNote extends Command {
    private final Conveyor conveyor;
    private final Amper amper;
    public ConveyorScoreNote() {
        conveyor = Conveyor.getInstance();
        amper = Amper.getInstance();
        addRequirements(conveyor, amper);
    };
    public void execute(){
        if (amper.hasNote()){
            amper.score();}
        else if (conveyor.isNoteAtShooter()) {
            conveyor.toShooter();
            }
        }
    }