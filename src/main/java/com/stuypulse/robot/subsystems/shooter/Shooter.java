/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.RobotType;
import com.stuypulse.robot.util.ShooterSpeeds;
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

    public Shooter() {
        leftTargetRPM = new SmartNumber("Shooter/Left Target RPM", 0);
        rightTargetRPM = new SmartNumber("Shooter/Right Target RPM", 0);
    }

    public final double getLeftTargetRPM() {
        return leftTargetRPM.get();
    }

    public final double getRightTargetRPM() {
        return rightTargetRPM.get();
    }

    public final void setTargetSpeeds(ShooterSpeeds speeds) {
        this.leftTargetRPM.set(speeds.getLeftRPM());
        this.rightTargetRPM.set(speeds.getRightRPM());
    }
    
    public final void stop() {
        setTargetSpeeds(new ShooterSpeeds());
    }

    public final boolean atTargetSpeeds() {
        return Math.abs(getLeftShooterRPM() - getLeftTargetRPM()) < Settings.Shooter.AT_RPM_EPSILON
            && Math.abs(getRightShooterRPM() - getRightTargetRPM()) < Settings.Shooter.AT_RPM_EPSILON;
    }

    public abstract double getLeftShooterRPM();

    public abstract double getRightShooterRPM();

    public final double getAverageShooterRPM() {
        return (getLeftShooterRPM() + getRightShooterRPM()) / 2.0;
    }

    public abstract boolean noteShot();
}
