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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class ConveyorImpl extends Conveyor {

    private final CANSparkMax gandalfMotor;
    private final CANSparkMax shooterFeederMotor;

    private final DigitalInput irSensor;

    private final BStream isAtShooter;

    protected ConveyorImpl() {
        gandalfMotor = new CANSparkMax(Ports.Conveyor.GANDALF, MotorType.kBrushless);
        shooterFeederMotor = new CANSparkMax(Ports.Conveyor.FEEDER, MotorType.kBrushless);

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
    public void toShooter() {
        gandalfMotor.set(+Settings.Conveyor.GANDALF_SHOOTER_SPEED.get());
        shooterFeederMotor.set(+Settings.Conveyor.FEEDER_SHOOTER_SPEED.get());
    }

    @Override
    public void toAmp() {
        gandalfMotor.set(-Settings.Conveyor.GANDALF_AMP_SPEED.get());
        shooterFeederMotor.set(+Settings.Conveyor.FEEDER_AMP_SPEED.get());
    }

    public void stop() {
        gandalfMotor.set(0);
        shooterFeederMotor.set(0);
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Conveyor/Gandalf Motor Current", gandalfMotor.getOutputCurrent());
        SmartDashboard.putNumber("Conveyor/Shooter Feeder Motor Current", shooterFeederMotor.getOutputCurrent());

        SmartDashboard.putNumber("Conveyor/Gandalf Motor Speed", gandalfMotor.get());
        SmartDashboard.putNumber("Conveyor/Shooter Feeder Motor Spped", shooterFeederMotor.get());

        SmartDashboard.putBoolean("Conveyor/Note At Shooter", isNoteAtShooter());
        SmartDashboard.putBoolean("Conveyor/Note At Shooter (Raw)", !irSensor.get());
    }
}
