package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.subsystems.conveyor.Conveyor;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ConveyorShootNoIntake extends InstantCommand {
    
    private final Conveyor conveyor;

    public ConveyorShootNoIntake() {
        conveyor = Conveyor.getInstance();

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.toShooter();
    }

}
