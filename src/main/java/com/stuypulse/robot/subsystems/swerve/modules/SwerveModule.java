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

    private SwerveModuleState targetState;

    public SwerveModule(String id, Translation2d offset) {
        this.id = id;
        this.offset = offset;

        targetState = new SwerveModuleState();
    }

    public final String getId() {
        return this.id;
    }

    public final Translation2d getModuleOffset() {
        return this.offset;
    }

    public abstract double getVelocity();
    public abstract Rotation2d getAngle();
    public abstract SwerveModulePosition getModulePosition();

    public final SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public final void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    public final SwerveModuleState getTargetState() {
        return targetState;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Target Speed", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Velocity", getVelocity());
    }
}
    

