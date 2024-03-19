/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.climber;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.numbers.IStream;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Operator;
import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.climber.Climber;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.util.ShooterSpeeds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDrive extends Command {

    private final Climber climber;
    private final IStream voltage;
    private final BStream shouldSafe;

    public ClimberDrive(Gamepad gamepad) {
        climber = Climber.getInstance();

        voltage = IStream.create(gamepad::getLeftY)
            .filtered(x -> {
                if (x > 0)
                    return +Operator.CLIMB_DRIVE_VOLTAGE_UP.get();
                else
                    return -Operator.CLIMB_DRIVE_VOLTAGE_DOWN.get();
            });
        
        shouldSafe = BStream.create(() -> gamepad.getLeftY() < -0.25)
            .and(() -> Amper.getInstance().getTargetHeight() < Lift.SAFE_CLIMB_HEIGHT);

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setVoltageOverride(voltage.get());

        SmartDashboard.putNumber("Climber/Gamepad Voltage", voltage.get());

        if (shouldSafe.get()) {
            Amper.getInstance().setTargetHeight(Settings.Amper.Lift.SAFE_CLIMB_HEIGHT);
            Shooter.getInstance().setTargetSpeeds(new ShooterSpeeds());
        }
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
