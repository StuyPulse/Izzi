/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.climber;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.numbers.IStream;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climber.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDrive extends Command {

    private final Climber climber;
    private final IStream voltage;

    public ClimberDrive(Gamepad gamepad) {
        climber = Climber.getInstance();

        voltage = IStream.create(gamepad::getLeftY)
            .filtered(x -> x * Settings.Operator.CLIMB_DRIVE_VOLTAGE.get());

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setVoltageOverride(voltage.get());

        SmartDashboard.putNumber("Climber/Gamepad Voltage", voltage.get());
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
