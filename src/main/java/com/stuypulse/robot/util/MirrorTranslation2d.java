package com.stuypulse.robot.util;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Translation2d;

public class MirrorTranslation2d {

    public static MirrorTranslation2d fromBlue(Translation2d blue) {
        return new MirrorTranslation2d(blue);
    }

    private final Translation2d red;
    private final Translation2d blue;

    private MirrorTranslation2d(Translation2d blue) {
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
