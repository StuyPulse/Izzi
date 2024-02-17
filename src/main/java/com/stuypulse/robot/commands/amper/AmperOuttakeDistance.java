/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.subsystems.amper.Amper;

import edu.wpi.first.wpilibj2.command.Command;

public class AmperOuttakeDistance extends Command {

    private final Amper amper;
    private final double distance;

    private double startingDistance;

    public AmperOuttakeDistance(double distance) {
        amper = Amper.getInstance();
        this.distance = distance;
        addRequirements(amper);
    }

    @Override
    public void initialize() {
        startingDistance = amper.getNoteDistance();
    }

    @Override
    public void execute() {
        amper.score();
    }

    @Override
    public void end(boolean interrupted) {
        amper.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return amper.getNoteDistance() >= distance + startingDistance;
    }
}
