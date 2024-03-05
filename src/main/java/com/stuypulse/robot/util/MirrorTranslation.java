package com.stuypulse.robot.util;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Translation2d;

public class MirrorTranslation {

    public static MirrorTranslation fromBlue(Translation2d blue) {
        return new MirrorTranslation(blue);
    }

    private final Translation2d red;
    private final Translation2d blue;

    private MirrorTranslation(Translation2d blue) {
        this.blue = blue;
        this.red = new Translation2d(blue.getX(), Field.WIDTH - blue.getY());
    }

    public Translation2d get() {
        if (Robot.isBlue()) {
            return blue;
        }

        return red;
    }

}
