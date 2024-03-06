package com.stuypulse.robot.util;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MirrorPose2d {

    private final boolean isBlue;
    private final Pose2d pose;
    private final Pose2d opposite;
    
    public MirrorPose2d(Alliance alliance, Pose2d pose) {
        this.pose = pose;
        
        isBlue = alliance == Alliance.Blue;
        opposite = Field.transformToOppositeAlliance(pose);
    }

    public Pose2d get() {
        return (isBlue == Robot.isBlue()) ? pose : opposite;
    }

    public Pose2d getRed() {
        return isBlue ? opposite : pose;
    }

    public Pose2d getBlue() {
        return isBlue ? pose : opposite;
    }

}
