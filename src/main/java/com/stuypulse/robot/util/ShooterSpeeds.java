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
        // double higher = shooterRPM + shooterDifferential / 2.0;
        // double lower = shooterRPM - shooterDifferential / 2.0;

        // if (robotPose.getY() > Field.getAllianceSpeakerPose().getY()) {
        //     rightRPM = higher;
        //     leftRPM = lower;
        // } else {
        //     rightRPM = lower;
        //     leftRPM = higher;
        // }
        
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
