package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ConveyorShootRoutine extends SequentialCommandGroup {

    public ConveyorShootRoutine() {
        this(Settings.Conveyor.SHOOT_WAIT_DELAY.get());
    }

    public ConveyorShootRoutine(double delay) {
        addCommands(
            new InstantCommand(SwerveDrive.getInstance()::stop, SwerveDrive.getInstance()),
            new ConveyorShoot(),
            new WaitCommand(delay),
            new ConveyorStop(),
            new IntakeStop()
        );
    }
    
}
