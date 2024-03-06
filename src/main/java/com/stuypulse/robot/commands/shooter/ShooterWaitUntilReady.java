package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShooterWaitUntilReady extends WaitUntilCommand {

    public ShooterWaitUntilReady() {
        super(() -> {
            Amper amper = Amper.getInstance();

            return Shooter.getInstance().atTargetSpeeds()
                && amper.getTargetHeight() == Lift.SHOOTING_HEIGHT
                && amper.isAtTargetHeight(Lift.MAX_SHOOTING_ERROR);
        });
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            CommandScheduler.getInstance().schedule(
                new LEDSet(LEDInstructions.PURPLE)
                    .withTimeout(1.0));
        }
    }
}
