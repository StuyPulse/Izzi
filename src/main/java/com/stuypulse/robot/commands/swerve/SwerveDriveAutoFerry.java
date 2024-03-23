package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class SwerveDriveAutoFerry extends SwerveDriveDriveAligned {

    private final Odometry odometry;
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final Intake intake;

    public SwerveDriveAutoFerry(Gamepad driver) {
        super(driver);

        shooter = Shooter.getInstance();
        odometry = Odometry.getInstance();
        conveyor = Conveyor.getInstance();
        intake = Intake.getInstance();

        addRequirements(shooter, odometry);
    }

    // returns pose of close amp corner
    private Translation2d getTargetPose() {
        return Robot.isBlue()
            ? new Translation2d(0, Field.WIDTH)
            : new Translation2d(0, 0);
    }

    @Override
    public Rotation2d getTargetAngle() {
        return getTargetPose().minus(odometry.getPose().getTranslation()).getAngle();
    }

    @Override
    public double getDistanceToTarget() {
        return getTargetPose().getDistance(odometry.getPose().getTranslation());
    }

    @Override
    public void execute() {
        super.execute();

        if (odometry.getPose().getX() < Field.FERRY_SHOT_THRESHOLD_X) {
            intake.acquire();
            conveyor.toShooter();
            shooter.setTargetSpeeds(Settings.Shooter.FERRY);
        } else {
            intake.stop();
            conveyor.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        conveyor.stop();
        shooter.setTargetSpeeds(Settings.Shooter.PODIUM_SHOT);
    }
}
