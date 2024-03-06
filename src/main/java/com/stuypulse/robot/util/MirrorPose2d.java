package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class MirrorPose2d {

    private final MirrorTranslation2d translation;
    private final MirrorRotation2d rotation;

    public MirrorPose2d(MirrorTranslation2d translation, MirrorRotation2d rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    public Pose2d get() {
        return new Pose2d(translation.get(), rotation.get());
    }
}
