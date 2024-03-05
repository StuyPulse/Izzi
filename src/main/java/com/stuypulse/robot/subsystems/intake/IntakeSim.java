/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.revrobotics.CANSparkBase.IdleMode;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSim extends Intake {

    private double intake_motor;
    private double conveyor_motor;

    private SmartBoolean intakeIR;

    public IntakeSim() {
        intake_motor = 0;
        conveyor_motor = 0;
        intakeIR = new SmartBoolean("Intake/Sim IR Value", false);
    }

    @Override
    public void acquire() {
        intake_motor = Settings.Intake.ACQUIRE_SPEED;
        conveyor_motor = Settings.Intake.ACQUIRE_SPEED;
    }

    @Override
    public void deacquire() {
        intake_motor = -Settings.Intake.DEACQUIRE_SPEED;
        conveyor_motor = -Settings.Intake.DEACQUIRE_SPEED;
    }

    @Override
    public void stop() {
        intake_motor = 0;
        conveyor_motor = 0;
    }

    @Override
    public double getIntakeRollerSpeed() {
        return intake_motor; // both running at same speed :)
    }

    @Override
    public boolean hasNote() {
        return intakeIR.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Intake/Intake Motor Speed", intake_motor);
        SmartDashboard.putNumber("Intake/Conveyor Motor Speed", conveyor_motor);
    }

    @Override
    public void setIdleMode(IdleMode mode) {
    }
}
