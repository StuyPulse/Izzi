package com.stuypulse.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractSwerveModule extends SubsystemBase {
    public abstract double getVelocity();
    public abstract Rotation2d getAngle();
    public abstract String getId();
    public abstract SwerveModuleState getState();
    public abstract Translation2d getModuleOffset();
    public abstract SwerveModulePosition getModulePosition();
    public abstract void setState(SwerveModuleState state);
    public abstract void periodic();
}
    

