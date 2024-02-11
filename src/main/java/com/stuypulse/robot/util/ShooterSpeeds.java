package com.stuypulse.robot.util;

public class ShooterSpeeds {
    
    public final Number leftRPM;
    public final Number rightRPM;

    public ShooterSpeeds(Number leftRPM, Number rightRPM) {
        this.leftRPM = leftRPM;
        this.rightRPM = rightRPM;
    }

    public ShooterSpeeds() {
        this(0, 0);
    }
}
