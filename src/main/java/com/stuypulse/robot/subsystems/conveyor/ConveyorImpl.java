/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.conveyor;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Motors.StatusFrame;
import com.stuypulse.robot.util.FilteredRelativeEncoder;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

public class ConveyorImpl extends Conveyor {

    private final CANSparkFlex gandalfMotor;

    private final RelativeEncoder gandalfEncoder;

    protected ConveyorImpl() {
        gandalfMotor = new CANSparkFlex(Ports.Conveyor.GANDALF, MotorType.kBrushless);

        gandalfEncoder = new FilteredRelativeEncoder(gandalfMotor);

        gandalfEncoder.setVelocityConversionFactor(1.0 / 2.0);

        Motors.disableStatusFrames(gandalfMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_POSIITION, StatusFrame.ABS_ENCODER_VELOCITY);

        Motors.Conveyor.GANDALF_MOTOR.configure(gandalfMotor);
    }

    @Override
    public double getGandalfSpeed() {
        return gandalfMotor.get();
    }

    @Override
    public void toShooter() {
        gandalfMotor.set(+Settings.Conveyor.GANDALF_SHOOTER_SPEED.get());
    }

    @Override
    public void toAmp() {
        gandalfMotor.set(-Settings.Conveyor.GANDALF_AMP_SPEED);
    }

    @Override
    public void stop() {
        gandalfMotor.stopMotor();
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        gandalfMotor.setIdleMode(mode);
        gandalfMotor.burnFlash();
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Conveyor/Gandalf Motor Current", gandalfMotor.getOutputCurrent());

        SmartDashboard.putNumber("Conveyor/Gandalf Motor Speed", gandalfMotor.get());

        SmartDashboard.putNumber("Conveyor/Gandalf RPM", gandalfEncoder.getVelocity());

        SmartDashboard.putNumber("Conveyor/Gandalf Linear Velocity", gandalfEncoder.getVelocity() * Units.inchesToMeters(1.0) * Math.PI);
    }
}
