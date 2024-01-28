package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.subsystems.conveyor.Conveyor;

import edu.wpi.first.wpilibj2.command.Command;

public class ConveyorShoot extends Command{

    private final Conveyor conveyor;

    public ConveyorShoot() {
        conveyor = Conveyor.getInstance();
        
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.toShooter();
    }
}
