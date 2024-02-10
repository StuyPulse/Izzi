package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveToChain extends Command {

    private final Odometry odometry;
    private final SwerveDrive swerve;

    private Pose2d trapPose;
    
    public SwerveDriveDriveToChain() {
        odometry = Odometry.getInstance();
        swerve = SwerveDrive.getInstance();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        trapPose = Field.getClosestAllianceTrapPose(odometry.getPose());
    }

    @Override
    public void execute() {
        Rotation2d translationAngle = trapPose.minus(odometry.getPose()).getTranslation().getAngle();
        Translation2d translation = new Translation2d(Alignment.INTO_CHAIN_SPEED.get(), translationAngle);

        swerve.drive(new Vector2D(translation), 0);
    }

    private double getDistanceToTrap() {
        return odometry.getPose().minus(trapPose).getTranslation().getNorm();
    }

    @Override
    public boolean isFinished() {
        return getDistanceToTrap() <= Alignment.TRAP_CLIMB_DISTANCE.get();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

}
