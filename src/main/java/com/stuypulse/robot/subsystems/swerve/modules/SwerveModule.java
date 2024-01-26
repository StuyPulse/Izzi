package com.stuypulse.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveModule extends SubsystemBase {

    private final String id;
    
    private final Translation2d offset;

    public SwerveModuleState targetState;

    public SwerveModule(String id, Translation2d offset) {
        this.id = id;
        this.offset = offset;

        targetState = new SwerveModuleState();
    }

    public String getId() {
        return this.id;
    }

    public Translation2d getModuleOffset() {
        return this.offset;
    }

    public abstract double getVelocity();
    public abstract Rotation2d getAngle();
    public abstract SwerveModulePosition getModulePosition();

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public void setState(SwerveModuleState state) {
        targetState = state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Swerve/Modules/" + this.getId() + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + this.getId() + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + this.getId()+ "/Target Speed", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Modules/" + this.getId() + "/Speed", getVelocity());
    }
}
    

