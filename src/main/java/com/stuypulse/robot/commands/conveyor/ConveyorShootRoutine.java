package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ConveyorShootRoutine extends SequentialCommandGroup {

    public static Command untilNoteShot(double timeout) {
        return new SequentialCommandGroup(
            new InstantCommand(SwerveDrive.getInstance()::stop, SwerveDrive.getInstance()),
            new ConveyorShoot(),
            new WaitUntilCommand(Shooter.getInstance()::noteShot)
                .withTimeout(timeout),
            new ConveyorStop(),
            new IntakeStop()
        );
    }

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
