package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;

public class AutonSetup extends SwerveDriveToPose {
    public AutonSetup(PathPlannerAuto auton) {
        super(PathPlannerAuto.getStaringPoseFromAutoFile(auton.getName()));
    }
}
