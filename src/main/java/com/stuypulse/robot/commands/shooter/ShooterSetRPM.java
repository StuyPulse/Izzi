package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterSetRPM extends InstantCommand {
    private final Shooter shooter;
    private final Number leftTargetRPM;
    private final Number rightTargetRPM;

    public ShooterSetRPM(Number leftTargetRPM, Number rightTargetRPM) {
        shooter = Shooter.getInstance();
        this.leftTargetRPM = leftTargetRPM;
        this.rightTargetRPM = rightTargetRPM;
        
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setLeftTargetRPM(leftTargetRPM);
        shooter.setRightTargetRPM(rightTargetRPM);
    }
    
}
    