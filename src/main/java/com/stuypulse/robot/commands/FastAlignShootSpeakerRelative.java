package com.stuypulse.robot.commands;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.util.MirrorRotation2d;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.Derivative;
import com.stuypulse.stuylib.streams.numbers.filters.TimedMovingAverage;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// needs secondary 
public class FastAlignShootSpeakerRelative extends Command {

    private final SwerveDrive swerve;
    private final Odometry odometry;

    private final Conveyor conveyor;
    private final Intake intake;

    private final HolonomicController controller;
    private final Supplier<Pose2d> poseSupplier;
    private final BStream isAligned;
    private final BStream isFinished;
    private final IStream velocityError;

    private final FieldObject2d targetPose2d;

    private double xTolerance;
    private double yTolerance;
    private double thetaTolerance;
    private double velocityTolerance;
    
    private Pose2d targetPose;

    public FastAlignShootSpeakerRelative(double angleToSpeaker) {
        this(angleToSpeaker, Alignment.PODIUM_SHOT_DISTANCE.get(), 0.75);
    }

    public FastAlignShootSpeakerRelative(double angleToSpeaker, double shootTimeout) {
        this(angleToSpeaker, Alignment.PODIUM_SHOT_DISTANCE.get(), shootTimeout);
    }

    public FastAlignShootSpeakerRelative(double angleToSpeaker, double distanceToSpeaker, double shootTimeout) {
        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();
        conveyor = Conveyor.getInstance();
        intake = Intake.getInstance();

        MirrorRotation2d angle = MirrorRotation2d.fromBlue(
            Rotation2d.fromDegrees(SLMath.clamp(
                angleToSpeaker, Alignment.PODIUM_SHOT_MAX_ANGLE)));

        double distance = SLMath.clamp(distanceToSpeaker, 1, 5);

        poseSupplier = () -> new Pose2d(
            Field.getAllianceSpeakerPose().getTranslation()
                .plus(new Translation2d(distance, angle.get())),
            angle.get());

        targetPose2d = odometry.getField().getObject("Target Pose");

        controller = new HolonomicController(
            new PIDController(Translation.kP, Translation.kI, Translation.kD),
            new PIDController(Translation.kP, Translation.kI, Translation.kD),
            new AnglePIDController(Rotation.kP, Rotation.kI, Rotation.kD));

        isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Both(Alignment.DEBOUNCE_TIME))
            .filtered(new BDebounce.Falling(2.0));

        isFinished = isAligned
            .filtered(new BDebounce.Rising(shootTimeout))
            .or(Shooter.getInstance()::noteShot);

        velocityError = IStream.create(() -> {
            ChassisSpeeds speeds = controller.getError();

            return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).getNorm();
        })
            .filtered(new TimedMovingAverage(0.05))
            .filtered(new LowPassFilter(0.05))
            .filtered(x -> Math.abs(x));

        xTolerance = 0.2;
        yTolerance = 0.2;
        thetaTolerance = 10;
        velocityTolerance = 0.2;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        targetPose = poseSupplier.get();
        SmartDashboard.putBoolean("AutonAlignment", true);
    }
    
    private boolean isAligned() {
        return controller.isDone(xTolerance, yTolerance, thetaTolerance)
            && velocityError.get() < velocityTolerance;
    }

    @Override
    public void execute() {
        targetPose2d.setPose(targetPose);
        controller.update(targetPose, odometry.getPose());

        Vector2D speed = new Vector2D(controller.getOutput().vxMetersPerSecond, controller.getOutput().vyMetersPerSecond)
            .clamp(Swerve.MAX_MODULE_SPEED);
        double rotation = SLMath.clamp(controller.getOutput().omegaRadiansPerSecond, Motion.MAX_ANGULAR_VELOCITY.get());
        
        SmartDashboard.putNumber("Alignment/Translation Target Speed", speed.distance());

        if (Math.abs(rotation) < Swerve.ALIGN_OMEGA_DEADBAND.get())
            rotation = 0;

        ChassisSpeeds clamped = new ChassisSpeeds(
            speed.x, speed.y, rotation);
        
        swerve.setChassisSpeeds(clamped);

        if (isAligned.get()) {
            conveyor.toShooter();
            intake.acquire();
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished.get() && isAligned.get();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        Field.clearFieldObject(targetPose2d);
        SmartDashboard.putBoolean("AutonAlignment", false);
    }


}
