package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class SwerveDriveAutoFerry extends SwerveDriveDriveAligned {

    private final SwerveDrive swerve;
    private final Odometry odometry;
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final Intake intake;

    public SwerveDriveAutoFerry(Gamepad driver) {
        super(driver);
        shooter = Shooter.getInstance();
        odometry = Odometry.getInstance();
        swerve = SwerveDrive.getInstance();
        conveyor = Conveyor.getInstance();
        intake = Intake.getInstance();

        addRequirements(shooter, odometry, swerve);
    }

    @Override
    public Rotation2d getTargetAngle() {
        return odometry.getPose().getRotation().minus(Field.getOpposingSourcePose().getRotation());
    }

    @Override
    public double getDistanceToTarget() {
        return odometry.getPose().getTranslation().getDistance(Field.getOpposingSourcePose().getTranslation());
    }

    @Override
    public void execute() {
        super.execute();
        Pose2d robotPose = odometry.getPose();
        if (robotPose.getX() > Field.getAllianceWingX() && robotPose.getX() < Field.getOpposingWingX()) {
            intake.acquire();
            conveyor.toShooter();
            shooter.setTargetSpeeds(Settings.Shooter.FERRY);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        conveyor.stop();
        shooter.stop(); 
    }
}
