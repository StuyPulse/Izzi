/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.conveyor;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ConveyorSim extends Conveyor {

    private double gandalfMotorSpeed;

    public ConveyorSim() {
        gandalfMotorSpeed = 0;
    }

    @Override
    public double getGandalfSpeed() {
        return gandalfMotorSpeed;
    }

    @Override
    public boolean isNoteAtShooter() {
        return gandalfMotorSpeed > 0;
    }

    @Override
    public void toShooter() {
        gandalfMotorSpeed = +Settings.Conveyor.GANDALF_SHOOTER_SPEED.get();
    }

    @Override
    public void toAmp() {
        gandalfMotorSpeed = -Settings.Conveyor.GANDALF_AMP_SPEED;
    }

    @Override
    public void stop() {
        gandalfMotorSpeed = 0;
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Conveyor/Gandalf Motor Speed", gandalfMotorSpeed);
    }
}
