/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class AmperToConveyor extends Command {

    private final Amper amper;
    private final Shooter shooter;

    public AmperToConveyor() {
        amper = Amper.getInstance();
        shooter = Shooter.getInstance();

        addRequirements(amper);
    }

    @Override
    public void initialize() {
        amper.toConveyor();
        shooter.setTargetSpeeds(Settings.Shooter.REVERSE);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTargetSpeeds(Settings.Shooter.PODIUM_SHOT);
    }

}
