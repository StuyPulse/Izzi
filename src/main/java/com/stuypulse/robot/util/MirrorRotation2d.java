package com.stuypulse.robot.util;

import com.stuypulse.robot.Robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class MirrorRotation2d {

    public static MirrorRotation2d fromBlue(Rotation2d blue) {
        return new MirrorRotation2d(blue);
    }

    private final Rotation2d red;
    private final Rotation2d blue;

    private MirrorRotation2d(Rotation2d blue) {
        this.blue = blue;
        this.red = blue.times(-1);
    }

    public Rotation2d get() {
        if (Robot.isBlue()){
            return blue;
        }

        return red;
    }
    
}
