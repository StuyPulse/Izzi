package com.stuypulse.robot.subsystems.swerve;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.robot.constants.Settings.Swerve.BackLeft;
import com.stuypulse.robot.constants.Settings.Swerve.BackRight;
import com.stuypulse.robot.constants.Settings.Swerve.FrontLeft;
import com.stuypulse.robot.constants.Settings.Swerve.FrontRight;
import com.stuypulse.robot.subsystems.swerve.modules.SimModule;
import com.stuypulse.robot.subsystems.swerve.modules.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractSwerveDrive extends SubsystemBase {
        
    private static final AbstractSwerveDrive instance;

    static {
        if (RobotBase.isSimulation()) {
            instance = new SwerveDrive(
                new SimModule(FrontRight.ID, FrontRight.MODULE_OFFSET),
                new SimModule(FrontLeft.ID, FrontLeft.MODULE_OFFSET),
                new SimModule(BackLeft.ID, BackLeft.MODULE_OFFSET),
                new SimModule(BackRight.ID, BackRight.MODULE_OFFSET)
            );
        }     
        else {
            instance = new SwerveDrive(  
                new SwerveModule(FrontRight.ID, FrontRight.MODULE_OFFSET, Ports.Swerve.FrontRight.TURN, FrontRight.ABSOLUTE_OFFSET, Ports.Swerve.FrontRight.DRIVE),
                new SwerveModule(FrontLeft.ID, FrontLeft.MODULE_OFFSET, Ports.Swerve.FrontLeft.TURN, FrontLeft.ABSOLUTE_OFFSET, Ports.Swerve.FrontLeft.DRIVE),
                new SwerveModule(BackLeft.ID, BackLeft.MODULE_OFFSET, Ports.Swerve.BackLeft.TURN, BackLeft.ABSOLUTE_OFFSET, Ports.Swerve.BackLeft.DRIVE),
                new SwerveModule(BackRight.ID, BackRight.MODULE_OFFSET, Ports.Swerve.BackRight.TURN, BackRight.ABSOLUTE_OFFSET, Ports.Swerve.BackRight.DRIVE)
            );
        }
    }

    public static AbstractSwerveDrive getInstance() {
        return instance;
    }

    public abstract void initFieldObject(Field2d field);

    public abstract SwerveDriveKinematics getKinematics();
    public abstract SwerveModuleState[] getModuleStates();
    public abstract SwerveModulePosition[] getModulePositions();
    public abstract Translation2d[] getModuleOffsets();
    public abstract ChassisSpeeds getChassisSpeeds();

    public abstract void setChassisSpeed(ChassisSpeeds speeds);
    public abstract void setModuleStates(SwerveModuleState[] states);
    public abstract void drive(Vector2D velocity, double Rotation2D);
    public abstract void stop(double Rotation2D); 

    public abstract Rotation2d getGyroAngle();
}   
