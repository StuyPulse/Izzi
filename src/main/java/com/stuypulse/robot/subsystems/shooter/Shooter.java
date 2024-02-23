/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings.RobotType;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {

    private static final Shooter instance;

    static {
        if (Robot.ROBOT == RobotType.IZZI) {
            instance = new ShooterImpl();
        } else {
            instance = new ShooterSim();
        }
    }

    public static Shooter getInstance() {
        return instance;
    }

    private final SmartNumber leftTargetRPM;
    private final SmartNumber rightTargetRPM;
    private final SmartNumber feederTargetRPM;

    public Shooter() {
        leftTargetRPM = new SmartNumber("Shooter/Left Target RPM", 0);
        rightTargetRPM = new SmartNumber("Shooter/Right Target RPM", 0);
        feederTargetRPM = new SmartNumber("Shooter/Feeder Target RPM", 0);
    }

    public final double getLeftTargetRPM() {
        return leftTargetRPM.get();
    }

    public final double getRightTargetRPM() {
        return rightTargetRPM.get();
    }

    public final double getFeederTargetRPM() {
        return feederTargetRPM.get();
    }

    public final void setLeftTargetRPM(Number leftTargetRPM) {
        this.leftTargetRPM.set(leftTargetRPM);
    }

    public final void setRightTargetRPM(Number rightTargetRPM) {
        this.rightTargetRPM.set(rightTargetRPM);
    }

    public final void setFeederTargetRPM(Number feederTargetRPM) {
        this.feederTargetRPM.set(feederTargetRPM);
    }

    public abstract void stop();

    public abstract double getLeftShooterRPM();

    public abstract double getRightShooterRPM();

    public abstract double getFeederRPM();
}
