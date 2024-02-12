package com.stuypulse.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrivePathFindTo {

    private Pose2d target;
    private PathConstraints constraints;
    private double endVelocity;
    private double endRotationDelay;

    public SwerveDrivePathFindTo(Pose2d target, PathConstraints constraints, double endVelocity, double endRotationDelay) {
        this.target = target;
        this.constraints = constraints;
        this.endVelocity = endVelocity;
        this.endRotationDelay = endRotationDelay;
    }

    public SwerveDrivePathFindTo(Pose2d target, PathConstraints constraints, double endVelocity) {
        this(target, constraints, endVelocity, 0);
    }
    
    public SwerveDrivePathFindTo(Pose2d target, PathConstraints constraints) {
        this(target, constraints, 0);
    }

    public SwerveDrivePathFindTo(Pose2d target) {
        this(target, Settings.Swerve.Motion.DEFAULT_CONSTRAINTS);
    }

    public Command get() {
        return AutoBuilder.pathfindToPose(target, constraints, endVelocity, endRotationDelay);
    }

    public SwerveDrivePathFindTo withConstraints(PathConstraints constraints) {
        this.constraints = constraints;
        return this;
    }

    public SwerveDrivePathFindTo withEndVelocity(double endVelocity) {
        this.endVelocity = endVelocity;
        return this;
    }

    public SwerveDrivePathFindTo withEndRotationDelay(double endRotationDelay) {
        this.endRotationDelay = endRotationDelay;
        return this;
    }
}
