package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveXMode extends Command {
    private final SwerveDrive swerve;

    public SwerveDriveXMode() {
        swerve = SwerveDrive.getInstance();
        addRequirements(swerve);
    } 

    @Override
    public void execute() {
        SwerveModuleState[] states = new SwerveModuleState[] {
            //{front right, front left, back right, back left}
            new SwerveModuleState(0,Rotation2d.fromDegrees(225)),
            new SwerveModuleState(0,Rotation2d.fromDegrees(315)),
            new SwerveModuleState(0,Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0,Rotation2d.fromDegrees(135))
        };
        swerve.setModuleStates(states);
    }
}
