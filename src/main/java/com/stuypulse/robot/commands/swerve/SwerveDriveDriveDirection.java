package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.vectors.VStream;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveDirection extends Command {
    
    private final SwerveDrive swerve;

    private final VStream direction;

    public SwerveDriveDriveDirection(VStream direction) {
        swerve = SwerveDrive.getInstance();

        this.direction = direction;
    }

    @Override
    public void execute() {
        swerve.setFieldRelativeSpeeds(new ChassisSpeeds(
            direction.get().x,
            direction.get().y,
            0));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

}
