package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climber.Climber;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.numbers.IStream;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDrive extends Command {
    
    private final Climber climber; 
    private final IStream voltage;

    public ClimberDrive(Gamepad gamepad) {
        climber = Climber.getInstance();

        voltage = IStream.create(gamepad::getRightY)
            .filtered(x -> x * Settings.Climber.MAX_DRIVE_VOLTAGE.get());

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setVoltageOverride(voltage.get());
        
        SmartDashboard.putNumber("Climber/Gamepad Voltage", voltage.get());
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
