/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.util.ShooterSpeeds;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterSetRPM extends InstantCommand {
    
    private final Shooter shooter;

    private final ShooterSpeeds speeds;

    public ShooterSetRPM(ShooterSpeeds speeds) {
        shooter = Shooter.getInstance();

        this.speeds = speeds;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setLeftTargetRPM(speeds.leftRPM);
        shooter.setRightTargetRPM(speeds.rightRPM);
        shooter.setFeederTargetRPM(speeds.feederRPM);
    }
}
