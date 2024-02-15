/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

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
