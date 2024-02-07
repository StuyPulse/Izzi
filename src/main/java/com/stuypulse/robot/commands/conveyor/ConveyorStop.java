package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.subsystems.conveyor.Conveyor;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ConveyorStop extends InstantCommand {

    private final Conveyor conveyor;

    public ConveyorStop() {
        conveyor = Conveyor.getInstance();
        
        addRequirements(conveyor);
    }
    
    @Override
    public void initialize() {
        conveyor.stop();
    }
    
}
