package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.numbers.IStream;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AmperLiftDrive extends Command {
    
    private final Amper amper; 
    private final IStream voltage;

    public AmperLiftDrive(Gamepad gamepad) {
        amper = Amper.getInstance();

        voltage = IStream.create(gamepad::getLeftY)
            .filtered(x -> x * Settings.Amper.Lift.MAX_DRIVE_VOLTAGE.get());

        addRequirements(amper);
    }

    @Override
    public void execute() {
        amper.setVoltageOverride(voltage.get());
        
        SmartDashboard.putNumber("Amper/Lift/Gamepad Voltage", voltage.get());
    }

    @Override
    public void end(boolean interrupted) {
        amper.setTargetHeight(amper.getLiftHeight());
    }
}
