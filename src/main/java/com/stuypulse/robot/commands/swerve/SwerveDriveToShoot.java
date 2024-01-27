package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.Fiducial;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
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

    public SwerveDriveToShoot() {
        swerve = SwerveDrive.getInstance();

        this.targetPose2d = Odometry.getInstance().getField().getObject("Target Pose");

        controller = new HolonomicController(
            new PIDController(Translation.P, Translation.I, Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new AnglePIDController(Rotation.P, Rotation.I, Rotation.D)
        );
    
        addRequirements(swerve);
    }

    private Pose2d getSpeakerTargetPose() {
        Vector2D speakerCenterVec = new Vector2D(Field.getAllianceSpeakerPose().getTranslation());
        Vector2D robotVec = new Vector2D(Odometry.getInstance().getPose().getTranslation());

        double speakerOpeningX = Units.inchesToMeters(13.6);
        // the distances between the robot and the target
        double Dx = speakerCenterVec.x - robotVec.x;
        double Dy = speakerCenterVec.y - robotVec.y;

        // the offset of the speakers opening width from center using similar triangles 
        double dy = (Dy / Dx) * speakerOpeningX; 

        // gets the new speaker target vector to aim at
        Vector2D speakerTargetVec= new Vector2D(Field.getAllianceSpeakerPose().getX(), Field.getAllianceSpeakerPose().getY() + dy);

        // gets the target vector away from the speaker to the target distance to shoot from
        Vector2D targetVec = speakerTargetVec.add(robotVec.sub(speakerTargetVec).normalize().mul(Units.inchesToMeters(Alignment.TARGET_DISTANCE_IN.get())));
        Rotation2d targetAngle = targetVec.getTranslation2d().minus(robotVec.getTranslation2d()).getAngle().plus(Rotation2d.fromDegrees(180));
        return new Pose2d(targetVec.x, targetVec.y, targetAngle);
    }

    @Override
    public void initialize() {
        targetPose = getSpeakerTargetPose();
    }

    @Override 
    public void execute() {
        Pose2d currentPose = Odometry.getInstance().getPose();
       
        controller.update(targetPose, currentPose);
        swerve.setChassisSpeeds(controller.getOutput());
        targetPose2d.setPose(targetPose);
    }

    @Override
    public boolean isFinished() {
        return controller.isDone(Alignment.X_TOLERANCE.get(), Alignment.Y_TOLERANCE.get(), Alignment.ANGLE_TOLERANCE.get());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

}
