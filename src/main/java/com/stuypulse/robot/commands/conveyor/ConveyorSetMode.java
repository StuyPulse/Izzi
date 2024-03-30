package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.conveyor.ConveyorMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ConveyorSetMode extends InstantCommand {
    
    public ConveyorSetMode(ConveyorMode mode) {
        super(() -> {
            Conveyor.getInstance().setMode(mode);
        });
    }

}
