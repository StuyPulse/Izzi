package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveToShoot extends Command {

    private final SwerveDrive swerve;
    private final Odometry odometry;

    private final PIDController distanceController;
    private final AnglePIDController angleController;

    private final BStream isAligned;

    private double distanceTolerance;
    private double angleTolerance;

    private final double targetDistance;

    public SwerveDriveToShoot() {
        this(Alignment.PODIUM_SHOT_DISTANCE);
    }
    
    public SwerveDriveToShoot(double targetDistance) {
        this.targetDistance = SLMath.clamp(targetDistance, 1, 5);

        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();

        distanceController = new PIDController(Translation.kP, Translation.kI, Translation.kD);
        
        angleController = new AnglePIDController(Rotation.kP, Rotation.kI, Rotation.kD);

        isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Rising(Alignment.DEBOUNCE_TIME));
        
        distanceTolerance = Alignment.X_TOLERANCE.get();
        angleTolerance = Alignment.ANGLE_TOLERANCE.get();
    }

    public SwerveDriveToShoot withTolerance(double distanceTolerance, double angleTolerance) {
        this.distanceTolerance = distanceTolerance;
        this.angleTolerance = angleTolerance;
        return this;
    }
    
    public SwerveDriveToShoot withDistanceConstants(double p, double i, double d) {
        distanceController.setPID(p, i, d);
        return this;
    }
    
    public SwerveDriveToShoot withRotationConstants(double p, double i, double d) {
        angleController.setPID(p, i, d);
        return this;
    }

    private boolean isAligned() {
        return distanceController.isDone(distanceTolerance)
            && angleController.isDoneDegrees(angleTolerance);
    }

    @Override
    public void execute() {
        Translation2d toSpeaker = Field.getAllianceSpeakerPose().getTranslation()
            .minus(odometry.getPose().getTranslation());
        
        double speed = -distanceController.update(targetDistance, toSpeaker.getNorm());
        double rotation = angleController.update(
            Angle.fromRotation2d(toSpeaker.getAngle()).add(Angle.k180deg),
            Angle.fromRotation2d(odometry.getPose().getRotation()));

        Translation2d speeds = new Translation2d(
            speed,
            toSpeaker.getAngle());

        swerve.setFieldRelativeSpeeds(
            new ChassisSpeeds(
                speeds.getX(),
                speeds.getY(),
                rotation));
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
