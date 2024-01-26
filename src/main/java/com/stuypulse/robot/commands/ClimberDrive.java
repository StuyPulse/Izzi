package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climber.Climber;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.numbers.IStream;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDrive extends Command {
    private final Climber climber; 
    private final IStream velocity;

    public ClimberDrive(Gamepad gamepad) {
        climber = Climber.getInstance();

        velocity = IStream.create(gamepad::getLeftY)
            .filtered(x -> x * Settings.Climber.VELOCITY_LIMIT.get());

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setVoltageOverride(velocity.get());
        
        SmartDashboard.putNumber("Climber/Gamepad Velocity", velocity.get());
    }

    @Override
    public void end(boolean interrupted) {
        climber.setTargetHeight(climber.getHeight());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
