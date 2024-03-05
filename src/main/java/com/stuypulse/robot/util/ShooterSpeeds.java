package com.stuypulse.robot.util;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;

public class ShooterSpeeds {

    private final Number shooterRPM;

    private double topShooterRPM;
    private double bottomShooterRPM;

    public ShooterSpeeds() {
        this(0);
    }


    public ShooterSpeeds(Number shooterRPM) {
        this.shooterRPM = shooterRPM;

        update(Odometry.getInstance().getPose());
    }

    public ShooterSpeeds update(Pose2d robotPose) {
        topShooterRPM = shooterRPM.doubleValue();
        bottomShooterRPM = shooterRPM.doubleValue();

        return this;
    }

    public double getTopShooterRPM() {
        return topShooterRPM;
    }

    public double getBottomShooterRPM() {
        return bottomShooterRPM;
    }

}
