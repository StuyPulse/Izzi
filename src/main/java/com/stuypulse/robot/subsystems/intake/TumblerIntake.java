/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors.CANSparkConfig;
import com.stuypulse.robot.util.FilteredRelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TumblerIntake extends Intake {

    private static final CANSparkConfig TOP_CONFIG = new CANSparkConfig(false, IdleMode.kBrake);
    private static final CANSparkConfig BOTTOM_CONFIG = new CANSparkConfig(true, IdleMode.kBrake);

    private final CANSparkMax topMotor;
    private final RelativeEncoder topEncoder;

    private final CANSparkMax bottomMotor;
    private final RelativeEncoder bottomEncoder;

    protected TumblerIntake() {
        topMotor = new CANSparkMax(20, MotorType.kBrushless);
        topEncoder = new FilteredRelativeEncoder(topMotor);

        bottomMotor = new CANSparkMax(21, MotorType.kBrushless);
        bottomEncoder = new FilteredRelativeEncoder(bottomMotor);

        TOP_CONFIG.configure(topMotor);
        BOTTOM_CONFIG.configure(bottomMotor);
    }

    @Override
    public void acquire() {
        topMotor.set(+1.0);
        bottomMotor.set(-1.0);
    }

    @Override
    public void deacquire() {
        topMotor.set(-1.0);
        bottomMotor.set(+1.0);
    }

    @Override
    public void stop() {
        topMotor.stopMotor();
        bottomMotor.stopMotor();
    }

    @Override
    public boolean hasNote() {
        return false;
    }

    @Override
    public double getIntakeRollerSpeed() {
        return topMotor.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Intake/Top Motor Speed", topMotor.get());
        SmartDashboard.putNumber("Intake/Bottom Motor Speed", bottomMotor.get());

        SmartDashboard.putNumber("Intake/Top Motor RPM", topEncoder.getVelocity());
        SmartDashboard.putNumber("Intake/Bottom Motor RPM", bottomEncoder.getVelocity());

        SmartDashboard.putNumber("Intake/Top Motor Current", topMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Bottom Motor Current", bottomMotor.getOutputCurrent());
    }

    @Override
    public void setIdleMode(IdleMode mode) {
    }
    
}
