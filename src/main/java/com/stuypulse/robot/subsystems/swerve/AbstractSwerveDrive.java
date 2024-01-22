package com.stuypulse.robot.subsystems.swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/*   
*  Fields: 
*    - swerveModules:  AbstractSwerveModule...
*    - kinematics:  SwerveDriveKinematics

*    Hardware Components:
*    - gyro: AHRS
*    - modules2D: FieldObject2D[]
*
*   Tasks:
*    - drive
*    - followDrive 
*    - trackingDrive
*    - aligning (Trap, Speaker, Amp)
*    - GTADrive (shooting while driving)
*
*   Methods:
*    + singleton
*    + initFieldObject(Field2D field): void
*    + getModulePositions(): Translation2D[]
*    + getModuleStates(): SwerveModuleStates[]
*    + getModuleOffsets(): Rotation2D[]
*    + getChassisSpeeds(): ChassisSpeed[]
*    + setModuleStates(SwerveModuleState... states): void
*    + setChassisSpeed(ChassisSpeed): void 
*    + drive(double, Rotation2D)
*    + stop(double, Rotation2D)
*
*  SwerveDrive.java
*   Methods:
*    - getGyroAngle(): Rotation2D
*    - getGyroYaw(): Rotation2D
*    - getGyroPitch(): Rotation2D
*    - getGyroRoll(): Rotation2D
*    - getKinematics(): SwerveDriveKinematics
*    + periodic(): void
*
*/
public abstract class AbstractSwerveDrive {
    
    private SwerveDriveKinematics kinematics;
    private AbstractSwerveModule[] swerveModules; 

    public abstract void initFieldObject(Field2d field);
    public abstract Translation2d getModuleStates();
    public abstract Rotation2d getModuleOffsets();
    public abstract void setChassisSpeed();
    public abstract void setModuleStates();
    public abstract void drive(double Rotation2D);
    public abstract void stop(double Rotation2D); 
    





    
}   
