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

    private final FlywheelSim leftWheel;
    private final FlywheelSim rightWheel;

    private final Controller leftController;
    private final Controller rightController;

    protected ShooterSim() {
        leftWheel = new FlywheelSim(DCMotor.getNEO(1), 1, Settings.Shooter.MOMENT_OF_INERTIA);
        rightWheel = new FlywheelSim(DCMotor.getNEO(1), 1, Settings.Shooter.MOMENT_OF_INERTIA);

        leftController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
            .add(new PIDController(PID.kP, PID.kI, PID.kD));
        rightController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
            .add(new PIDController(PID.kP, PID.kI, PID.kD));
    }

    @Override
    public void stop() {
        leftWheel.setInputVoltage(0);
        rightWheel.setInputVoltage(0);
    }

    @Override
    public double getLeftShooterRPM() {
        return leftWheel.getAngularVelocityRPM();
    }

    @Override
    public double getRightShooterRPM() {
        return rightWheel.getAngularVelocityRPM();
    }

    @Override
    public void periodic() {
        super.periodic();

        leftController.update(getLeftTargetRPM(), getLeftShooterRPM());
        rightController.update(getRightTargetRPM(), getRightShooterRPM());

        leftWheel.setInputVoltage(leftController.getOutput());
        rightWheel.setInputVoltage(rightController.getOutput());

        SmartDashboard.putNumber("Shooter/Right RPM", getRightShooterRPM());
        SmartDashboard.putNumber("Shooter/Left RPM", getLeftShooterRPM());

        SmartDashboard.putNumber("Shooter/Left Voltage", leftController.getOutput());
        SmartDashboard.putNumber("Shooter/Right Voltage", rightController.getOutput());
    }

    @Override
    public void simulationPeriodic() {
        leftWheel.update(Settings.DT);
        rightWheel.update(Settings.DT);
    }
}
