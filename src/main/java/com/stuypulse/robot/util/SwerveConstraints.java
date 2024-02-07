package com.stuypulse.robot.util;

public class SwerveConstraints {
    public double maxDriveVel;
    public double maxDriveAccel;
    public double maxTurnVel;
    
    public SwerveConstraints(double maxDriveVel, double maxDriveAccel, double maxTurnVel) {
        this.maxDriveVel = maxDriveVel;
        this.maxDriveAccel = maxDriveAccel;
        this.maxTurnVel = maxTurnVel;
    }
}
