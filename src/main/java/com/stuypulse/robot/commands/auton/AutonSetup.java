/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class AutonSetup extends SwerveDriveToPose {
    public AutonSetup(PathPlannerAuto auton) {
        super(PathPlannerAuto.getStaringPoseFromAutoFile(auton.getName()));
    }
}
