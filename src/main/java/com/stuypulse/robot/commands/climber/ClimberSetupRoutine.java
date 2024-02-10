package com.stuypulse.robot.commands.climber;
import com.stuypulse.robot.commands.amper.AmperToHeight;
import com.stuypulse.robot.commands.conveyor.ConveyorToAmp;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveToChain;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimberSetupRoutine extends SequentialCommandGroup {

    private Pose2d getTargetPose() {
        Pose2d closestTrap = Field.getClosestAllianceTrapPose(Odometry.getInstance().getPose());
        Translation2d offsetTranslation = new Translation2d(Alignment.TRAP_SETUP_DISTANCE.get(), closestTrap.getRotation());

        return closestTrap.plus(new Transform2d(offsetTranslation, new Rotation2d()));
    }

    public ClimberSetupRoutine() {
        addCommands(
            // raise everything and get in position
            new ParallelCommandGroup(
                new AmperToHeight(Settings.Amper.Lift.MIN_HEIGHT).andThen(new ConveyorToAmp()),
                new ClimberToTop(),
                new SwerveDriveToPose(() -> getTargetPose())
                    .withTolerance(Units.inchesToMeters(3.0), Units.inchesToMeters(3.0), 3.0)
            ),
            // drive into chain
            new SwerveDriveDriveToChain()
        );
    }
}