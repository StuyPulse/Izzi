package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.util.ShooterSpeeds;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterSetRPM extends InstantCommand {
    private final Shooter shooter;
    private final ShooterSpeeds targetSpeeds;

    public ShooterSetRPM(ShooterSpeeds speeds) {
        shooter = Shooter.getInstance();
        this.targetSpeeds = speeds;
        
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setLeftTargetRPM(targetSpeeds.leftRPM);
        shooter.setRightTargetRPM(targetSpeeds.rightRPM);
    }
    
}
    