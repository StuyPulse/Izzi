package com.stuypulse.robot.util.vision;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.util.MirrorTranslation;

import edu.wpi.first.math.geometry.Pose2d;

public class MirrorPose {
    
    private final Pose2d red;

    private final Pose2d blue;

    public MirrorPose(Pose2d blue){
        this.blue = blue;
        this.red = new Pose2d(MirrorTranslation.fromBlue(blue.getTranslation()).get(), MirrorRotation.fromBlue(blue.getRotation()).get());
    }

    public Pose2d get(){
        if (Robot.isBlue()){
            return blue;
        }
        return red;
    }
}
