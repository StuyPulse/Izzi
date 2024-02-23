package com.stuypulse.robot.util;

public class ShooterSpeeds {

    public final double leftRPM;
    public final double rightRPM;
    public final double feederRPM;

    public ShooterSpeeds() {
        this(0, 0);
    }

    public ShooterSpeeds(Number shooterRPM, Number feederRPM) {
        this(shooterRPM.doubleValue(), feederRPM.doubleValue());
    }

    public ShooterSpeeds(double shooterRPM, double feederRPM) {
        this(shooterRPM, shooterRPM, feederRPM);
    }

    public ShooterSpeeds(Number leftRPM, Number rightRPM, Number feederRPM) {
        this(leftRPM.doubleValue(), rightRPM.doubleValue(), feederRPM.doubleValue());
    }
    
    public ShooterSpeeds(double leftRPM, double rightRPM, double feederRPM) {
        this.leftRPM = leftRPM;
        this.rightRPM = rightRPM;
        this.feederRPM = feederRPM;
    }

}
