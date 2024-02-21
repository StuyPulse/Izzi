/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.conveyor;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class ConveyorImpl extends Conveyor {

    private final CANSparkMax gandalfMotor;
    private final CANSparkMax shooterFeederMotor;

    private final RelativeEncoder gandalfEncoder;
    private final RelativeEncoder feederEncoder;

    private final DigitalInput irSensor;

    private final BStream isAtShooter;

    protected ConveyorImpl() {
        gandalfMotor = new CANSparkMax(Ports.Conveyor.GANDALF, MotorType.kBrushless);
        shooterFeederMotor = new CANSparkMax(Ports.Conveyor.FEEDER, MotorType.kBrushless);

        gandalfEncoder = gandalfMotor.getEncoder();
        feederEncoder = shooterFeederMotor.getEncoder();

        gandalfEncoder.setPositionConversionFactor(0.5);
        gandalfEncoder.setVelocityConversionFactor(0.5);

        feederEncoder.setPositionConversionFactor(1.0);
        feederEncoder.setVelocityConversionFactor(1.0);

        irSensor = new DigitalInput(Ports.Conveyor.IR_SENSOR);

        isAtShooter = BStream.create(irSensor).not()
            .filtered(new BDebounce.Rising(Settings.Conveyor.DEBOUNCE_TIME));

        Motors.Conveyor.GANDALF_MOTOR.configure(gandalfMotor);
        Motors.Conveyor.SHOOTER_FEEDER_MOTOR.configure(shooterFeederMotor);
    }

    @Override
    public double getGandalfSpeed() {
        return gandalfMotor.get();
    }

    @Override
    public double getFeederSpeed() {
        return shooterFeederMotor.get();
    }

    @Override
    public boolean isNoteAtShooter() {
        return isAtShooter.get();
    }

    @Override
    public void shoot() {
        gandalfMotor.set(+Settings.Conveyor.GANDALF_SHOOTER_SPEED.get());
        shooterFeederMotor.set(+Settings.Conveyor.FEEDER_SHOOT_SPEED.get());
    }

    @Override
    public void toShooter() {
        gandalfMotor.set(+Settings.Conveyor.GANDALF_SHOOTER_SPEED.get());
        shooterFeederMotor.set(+Settings.Conveyor.FEEDER_SHOOTER_SPEED.get());
    }

    @Override
    public void toAmp() {
        gandalfMotor.set(-Settings.Conveyor.GANDALF_AMP_SPEED);
        shooterFeederMotor.set(+Settings.Conveyor.FEEDER_AMP_SPEED.get());
    }

    @Override
    public void stopGandalf() {
        gandalfMotor.set(0);
    }

    @Override
    public void stopFeeder() {
        shooterFeederMotor.set(0);
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Conveyor/Gandalf Motor Current", gandalfMotor.getOutputCurrent());
        SmartDashboard.putNumber("Conveyor/Shooter Feeder Motor Current", shooterFeederMotor.getOutputCurrent());

        SmartDashboard.putNumber("Conveyor/Gandalf Motor Speed", gandalfMotor.get());
        SmartDashboard.putNumber("Conveyor/Shooter Feeder Motor Spped", shooterFeederMotor.get());

        SmartDashboard.putNumber("Conveyor/Feeder RPM", feederEncoder.getVelocity());
        SmartDashboard.putNumber("Conveyor/Gandalf RPM", gandalfEncoder.getVelocity());

        SmartDashboard.putNumber("Conveyor/Feeder Linear Velocity", feederEncoder.getVelocity() * Units.inchesToMeters(2.25) * Math.PI);
        SmartDashboard.putNumber("Conveyor/Gandalf Linear Velocity", gandalfEncoder.getVelocity() * Units.inchesToMeters(1.0) * Math.PI);

        SmartDashboard.putBoolean("Conveyor/Note At Shooter", isNoteAtShooter());
        SmartDashboard.putBoolean("Conveyor/Note At Shooter (Raw)", !irSensor.get());
    }
}
