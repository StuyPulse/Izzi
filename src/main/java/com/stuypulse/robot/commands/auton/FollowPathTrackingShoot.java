package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.conveyor.ConveyorShoot;
import com.stuypulse.robot.commands.conveyor.ConveyorStop;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.shooter.ShooterPodiumShot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDerivative;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FollowPathTrackingShoot extends SequentialCommandGroup {

    private final Odometry odometry;

    private final VStream robotPose;
    private final VStream robotVelocity;
    private final VStream projectedRobotPose;

    private double targetDistance;
    private double distanceTolerance;

    public static boolean shouldShoot() {
        Translation2d speaker = Field.getAllianceSpeakerPose().getTranslation();
        double distance = speaker
            // .minus(projectedRobotPose.get().getTranslation2d())
            .minus(Odometry.getInstance().getPose().getTranslation())
            .getNorm();

        // return Math.abs(targetDistance - distance) < distanceTolerance;
        return distance > 2.25;
    }
    
    public FollowPathTrackingShoot(PathPlannerPath path) {
        odometry = Odometry.getInstance();
        
        robotPose = VStream.create(() -> new Vector2D(odometry.getPose().getTranslation()));

        robotVelocity = robotPose
            .filtered(new VDerivative())
            .filtered(new VLowPassFilter(Alignment.PROJECTED_POSE_RC));

        projectedRobotPose = robotVelocity
            .filtered(v -> v.mul(Alignment.NOTE_TO_GOAL_TIME))
            .add(robotPose);
        
        distanceTolerance = 0.033;
        targetDistance = Alignment.PODIUM_SHOT_DISTANCE.get();
        
        addCommands(
            new ShooterPodiumShot(),
            new ShooterWaitForTarget(),
            new ParallelCommandGroup(
                SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(path),
                new SequentialCommandGroup(
                    new WaitUntilCommand(FollowPathTrackingShoot::shouldShoot),
                    new ConveyorShoot(),
                    new WaitCommand(Settings.Conveyor.SHOOT_WAIT_DELAY.get()),
                    new ConveyorStop(),
                    new IntakeStop()
                )
            )
        );
    }

    public FollowPathTrackingShoot withTolerance(double distanceTolerance) {
        this.distanceTolerance = distanceTolerance;
        return this;
    }

    public FollowPathTrackingShoot withTargetDistance(double target) {
        this.targetDistance = target;
        return this;
    }

}
