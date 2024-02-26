package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveDirection extends Command {
    
    private final SwerveDrive swerve;

    private final Vector2D direction;

    public SwerveDriveDriveDirection(Vector2D direction) {
        swerve = SwerveDrive.getInstance();

        this.direction = direction;
    }

    @Override
    public void execute() {
        swerve.setFieldRelativeSpeeds(new ChassisSpeeds(
            direction.x,
            direction.y,
            0));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

}
