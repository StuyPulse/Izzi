package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSetpointGenerator {
    private final SwerveDriveKinematics kinematics;
    
    public SwerveSetpointGenerator(final SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
    }

    public ChassisSpeeds generateSetpoint(ChassisSpeeds prevSetpoint, ChassisSpeeds desiredState) {
        if (Math.hypot(desiredState.vxMetersPerSecond, desiredState.vyMetersPerSecond) < Swerve.MODULE_VELOCITY_DEADBAND.get()) {
            return new ChassisSpeeds();
        }

        SwerveConstraints constraints = Swerve.CONSTRAINTS;

        // Enforce drive acceleration limits
        double dvMax = constraints.maxDriveAccel * Settings.DT;
        double dx = desiredState.vxMetersPerSecond - prevSetpoint.vxMetersPerSecond;
        double dy = desiredState.vyMetersPerSecond - prevSetpoint.vyMetersPerSecond;
        double dvAngle = Math.atan2(dy, dx);

        //  Enforce turn Limits
        double dtheta = desiredState.omegaRadiansPerSecond - prevSetpoint.omegaRadiansPerSecond;
        double maxAngleStep = Math.min(Math.abs(dtheta), constraints.maxTurnVel);

        ChassisSpeeds setpoint = new ChassisSpeeds(
            prevSetpoint.vxMetersPerSecond + dvMax * Math.cos(dvAngle),
            prevSetpoint.vyMetersPerSecond + dvMax * Math.sin(dvAngle),
            prevSetpoint.omegaRadiansPerSecond + maxAngleStep * Math.signum(dtheta)
        );

        // Enforce drive velocity limits
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(setpoint);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, constraints.maxDriveVel);
        setpoint = kinematics.toChassisSpeeds(moduleStates);

        SmartDashboard.putNumber("Swerve/Setpoint Generation/Setpoint X Vel", setpoint.vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Setpoint Generation/Setpoint Y Vel", setpoint.vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Setpoint Generation/Setpoint Theta Vel", setpoint.vxMetersPerSecond);

        return setpoint;
    }
}
