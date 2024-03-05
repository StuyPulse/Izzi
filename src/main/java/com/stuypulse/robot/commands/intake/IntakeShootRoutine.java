package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeShootRoutine extends SequentialCommandGroup {

    public IntakeShootRoutine() {
        this(Settings.Conveyor.SHOOT_WAIT_DELAY.get());
    }

    public IntakeShootRoutine(double delay) {
        addCommands(
                new IntakeShoot(),
                new WaitCommand(delay),
                new IntakeStop());
    }

}
