package com.stuypulse.robot.subsystems.swerve;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractSwerveDrive extends SubsystemBase {
    public abstract void initFieldObject(Field2d field);
    public abstract SwerveModuleState[] getModuleStates();
    public abstract Translation2d[] getModulePositions();
    public abstract void setChassisSpeed(ChassisSpeeds speeds);
    public abstract void setModuleStates(SwerveModuleState[] states);
    public abstract void drive(Vector2D velocity, double Rotation2D);
    public abstract void stop(double Rotation2D); 
}   
