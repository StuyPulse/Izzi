package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ConveyorShootRoutine extends SequentialCommandGroup {

    public ConveyorShootRoutine() {
        addCommands(
            new ConveyorToShooter(),
            new WaitCommand(Settings.Conveyor.AT_FEEDER_WAIT_DELAY.get()),
            new ConveyorShoot(),
            new WaitCommand(Settings.Conveyor.SHOOT_WAIT_DELAY.get()),
            new ConveyorStop()
        );
    }
    
}
