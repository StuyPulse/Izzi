package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveDriveAmpAlign extends SequentialCommandGroup {

    private static Pose2d getTargetPose(double distanceToWall) {
        boolean isBlue = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

        Translation2d amp = Field.getAllianceAmpPose().getTranslation();
        Translation2d delta = new Translation2d(0, isBlue ? -distanceToWall : distanceToWall);
        Rotation2d targetAngle = Rotation2d.fromDegrees(isBlue ? 270 : 90);

        return new Pose2d(amp.plus(delta), targetAngle);
    }

    public SwerveDriveAmpAlign() {
        addCommands(
            new SwerveDriveToPose(() -> getTargetPose(Alignment.AMP_WALL_SETUP_DISTANCE.get()))
                .withTolerance(Units.inchesToMeters(2.0), Units.inchesToMeters(4.0), 5),
            
            new SwerveDriveToPose(() -> getTargetPose(Alignment.AMP_WALL_SCORE_DISTANCE.get()))
                .withTolerance(Units.inchesToMeters(0.75), Units.inchesToMeters(1.0), 2)
        );
    }
    
}
