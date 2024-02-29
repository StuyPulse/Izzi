package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;

public class ShooterSpeeds {

    private final double shooterRPM;
    private final double shooterDifferential;
    private final double feederRPM;

    private double leftRPM;
    private double rightRPM;

    public ShooterSpeeds() {
        this(0, 0);
    }

    public ShooterSpeeds(Number shooterRPM, Number feederRPM) {
        this(shooterRPM.doubleValue(), feederRPM.doubleValue());
    }

    public ShooterSpeeds(double shooterRPM, double feederRPM) {
        this(shooterRPM, 0, feederRPM);
    }

    public ShooterSpeeds(Number shooterRPM, Number shooterDifferential, Number feederRPM) {
        this(shooterRPM.doubleValue(), shooterDifferential.doubleValue(), feederRPM.doubleValue());
    }
    
    public ShooterSpeeds(double shooterRPM, double shooterDifferential, double feederRPM) {
        this.shooterRPM = shooterRPM;
        this.shooterDifferential = shooterDifferential;
        this.feederRPM = feederRPM;

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
        
        leftRPM = shooterRPM + shooterDifferential / 2.0;
        rightRPM = shooterRPM - shooterDifferential / 2.0;

        return this;
    }

    public double getLeftRPM() {
        return leftRPM;
    }

    public double getRightRPM() {
        return rightRPM;
    }

    public double getFeederRPM() {
        return feederRPM;
    }
}
