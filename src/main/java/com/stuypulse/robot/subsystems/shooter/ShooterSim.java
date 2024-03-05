/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Shooter.Feedforward;
import com.stuypulse.robot.constants.Settings.Shooter.PID;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSim extends Shooter {

    private final FlywheelSim topWheel;
    private final FlywheelSim bottomWheel;

    private final Controller topController;
    private final Controller bottomController;

    protected ShooterSim() {
        topWheel = new FlywheelSim(DCMotor.getNEO(1), 1, Settings.Shooter.MOMENT_OF_INERTIA);
        bottomWheel = new FlywheelSim(DCMotor.getNEO(1), 1, Settings.Shooter.MOMENT_OF_INERTIA);

        topController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
            .add(new PIDController(PID.kP, PID.kI, PID.kD));
        bottomController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
            .add(new PIDController(PID.kP, PID.kI, PID.kD));
    }

    @Override
    public double getTopShooterRPM() {
        return topWheel.getAngularVelocityRPM();
    }

    @Override
    public double getBottomShooterRPM() {
        return bottomWheel.getAngularVelocityRPM();
    }

    @Override
    public boolean noteShot() {
        return atTargetSpeeds();
    }

    @Override
    public void periodic() {
        super.periodic();

        topController.update(getTopTargetRPM(), getTopShooterRPM());
        bottomController.update(getBottomTargetRPM(), getBottomShooterRPM());

        if (getTopTargetRPM() == 0 && getBottomTargetRPM() == 0) {
            topWheel.setInputVoltage(0);
            bottomWheel.setInputVoltage(0);
        } else {
            topWheel.setInputVoltage(topController.getOutput());
            bottomWheel.setInputVoltage(bottomController.getOutput());
        }

        SmartDashboard.putNumber("Shooter/Top RPM", getTopShooterRPM());
        SmartDashboard.putNumber("Shooter/Bottom RPM", getBottomShooterRPM());

        SmartDashboard.putNumber("Shooter/Top Voltage", topController.getOutput());
        SmartDashboard.putNumber("Shooter/Bottom Voltage", bottomController.getOutput());
    }

    @Override
    public void simulationPeriodic() {
        topWheel.update(Settings.DT);
        bottomWheel.update(Settings.DT);
    }
}
