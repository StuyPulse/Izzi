package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShooterWaitForTarget extends WaitUntilCommand {

    public ShooterWaitForTarget() {
        super(() -> Shooter.getInstance().atTargetSpeeds());
    }

}
