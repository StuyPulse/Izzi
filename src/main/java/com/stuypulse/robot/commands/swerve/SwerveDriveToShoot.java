package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveToShoot extends Command {
    /*
     * swerve
     * holonomi controller
     * targetpose pose2d
     * targetPose2d as a field object 2d
     * 
     * contrusctor (init them, add requirements)
     * initialize (do the math to get the target pose)
     * execute (set the target pose to the controller and then setChassisSpeeds to it)
     * isFinished (check if the controller is done within the tolerances)
     * end (stop the swerve)    
     * 
     */
    private final SwerveDrive swerve;
    private Pose2d targetPose;
    private final HolonomicController controller;
    private final FieldObject2d targetPose2d;

    public SwerveDriveToShoot(Pose2d targetPose) {
        swerve = SwerveDrive.getInstance();
        this.targetPose = targetPose;

        this.targetPose2d = Odometry.getInstance().getField().getObject("Target Pose");

        controller = new HolonomicController(
            new PIDController(Translation.P, Translation.I, Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new AnglePIDController(Rotation.P, Rotation.I, Rotation.D)
        );
    
        addRequirements(swerve);
    }

    private double linearInterpolate(double lowerY, double upperY, double lowerBound, double upperBound, double input) {
        if (input == lowerBound) {
            return lowerY;
        } else if (input == upperBound) {
            return upperY;
        } else if (input < lowerBound || input > upperBound) {
            throw new IllegalArgumentException("Input must be between lowerBound and upperBound");
        }
        else {
            return (upperY - lowerY) * (input - lowerBound) / (upperBound - lowerBound) + lowerY;
        }
    }

    //TODO: Make this work lmao
    private Pose2d getSpeakerTargetPose(Rotation2d angleToSpeaker) {
        //Everything under is from aiming at center of speaker
        Vector2D robotPose = new Vector2D(Odometry.getInstance().getPose().getTranslation());
        Vector2D targetPose = new Vector2D(this.targetPose.getTranslation());
        Vector2D targetVector = robotPose.add(robotPose.sub(targetPose).normalize().mul(Alignment.TARGET_DISTANCE_IN.get()));
        //Rotation2d angleToSpeaker = new Rotation2d(Math.atan2(targetPose.y - robotPose.y, targetPose.x - robotPose.x));

        double speakerOpeningLength = Units.inchesToMeters(41.625);
        Rotation2d speakerTargetAngle = Rotation2d.fromDegrees(linearInterpolate(-speakerOpeningLength / 2, speakerOpeningLength / 2, -70, 70, angleToSpeaker.getDegrees()));

        return new Pose2d(targetVector.x, targetVector.y, speakerTargetAngle);
    }

    @Override
    public void initialize() {
        Vector2D robotPose = new Vector2D(Odometry.getInstance().getPose().getTranslation());
        Vector2D targetPose = new Vector2D(this.targetPose.getTranslation());
        Rotation2d angleToSpeaker = new Rotation2d(Math.atan2(targetPose.y - robotPose.y, targetPose.x - robotPose.x));
        Vector2D targetVector = robotPose.add(robotPose.sub(targetPose).normalize().mul(Alignment.TARGET_DISTANCE_IN.get()));
        //this.targetPose = getSpeakerTargetPose(angleToSpeaker);
        
        this.targetPose = new Pose2d(targetVector.x, targetVector.y, angleToSpeaker);
    }

    @Override 
    public void execute() {
        Pose2d currentPose = Odometry.getInstance().getPose();
        targetPose2d.setPose(targetPose);

        controller.update(targetPose, currentPose);
        swerve.setChassisSpeeds(controller.getOutput());
    }

    
    @Override
    public boolean isFinished() {
        return controller.isDone(Alignment.X_TOLERANCE.get(), Alignment.Y_TOLERANCE.get(), Alignment.ANGLE_TOLERANCE.get());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        targetPose2d.setPose(Double.NaN, Double.NaN, new Rotation2d(Double.NaN));
    }

}
