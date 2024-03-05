package com.stuypulse.robot.util.vision;

import com.stuypulse.robot.Robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class MirrorRotation {

    public static MirrorRotation fromBlue(Rotation2d blue) {
        return new MirrorRotation(blue);
    }

    private final Rotation2d red;
    private final Rotation2d blue;

    private MirrorRotation(Rotation2d blue) {
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
