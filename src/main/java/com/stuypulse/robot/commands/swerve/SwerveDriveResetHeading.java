package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveResetHeading extends InstantCommand {
    
    public SwerveDriveResetHeading(Rotation2d heading) {
        super(() -> {
            Odometry o = Odometry.getInstance();
            o.reset(new Pose2d(o.getTranslation(), heading));
        });
    }

    public SwerveDriveResetHeading() {
        this(new Rotation2d());
    }

}
