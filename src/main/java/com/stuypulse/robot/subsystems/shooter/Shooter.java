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

    private final SmartNumber topTargetRPM;
    private final SmartNumber bottomTargetRPM;

    public Shooter() {
        topTargetRPM = new SmartNumber("Shooter/Top Motor RPM", 0);
           bottomTargetRPM = new SmartNumber("Shooter/Bottom Motor RPM", 0);
    }

    public final void setTargetSpeeds(ShooterSpeeds speeds) {
        topTargetRPM.set(speeds.getTopShooterRPM());
        bottomTargetRPM.set(speeds.getBottomShooterRPM());
    }

    public double getTopTargetRPM() {
        return topTargetRPM.get();
    }

    public double getBottomTargetRPM() {
        return bottomTargetRPM.get();
    }

    public abstract double getTopShooterRPM();

    public abstract double getBottomShooterRPM();

    public final double getAverageShooterRPM() {
        return (getTopShooterRPM() + getBottomShooterRPM()) / 2.0;
    }
    
    public final void stop() {
        setTargetSpeeds(new ShooterSpeeds());
    }

    public final boolean atTargetSpeeds() {
        return Math.abs(getTopTargetRPM() - getTopShooterRPM()) < Settings.Shooter.AT_RPM_EPSILON
            && Math.abs(getBottomTargetRPM() - getBottomShooterRPM()) < Settings.Shooter.AT_RPM_EPSILON;
    }

    public abstract boolean noteShot();
}
