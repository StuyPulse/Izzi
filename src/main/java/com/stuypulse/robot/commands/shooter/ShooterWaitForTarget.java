package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShooterWaitForTarget extends WaitUntilCommand {

    public ShooterWaitForTarget() {
        super(() -> Shooter.getInstance().atTargetSpeeds());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        if (interrupted) {
            CommandScheduler.getInstance().schedule(
                new LEDSet(LEDInstructions.PURPLE)
                    .withTimeout(1.0));
        }
    }
}
