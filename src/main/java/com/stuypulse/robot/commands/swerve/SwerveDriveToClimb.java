package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveToClimb extends Command {
    
    private final SwerveDrive swerve;
    private final Odometry odometry;
    
    private final HolonomicController controller;
    
    private final FieldObject2d targetPose2d;
    
    private Pose2d targetPose;

    private final double distance;

    public SwerveDriveToClimb() {
        this(Alignment.TRAP_SETUP_DISTANCE.get());
    }

    public SwerveDriveToClimb(double distance) {
        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();
        
        this.distance = distance;

        targetPose2d = odometry.getField().getObject("Target Pose");

        controller = new HolonomicController(
            new PIDController(Translation.kP, Translation.kI, Translation.kD),
            new PIDController(Translation.kP, Translation.kI, Translation.kD),
            new AnglePIDController(Rotation.kP, Rotation.kI, Rotation.kD));
    }
    
    private Pose2d getTargetPose() {
        Pose2d closestTrap = Field.getClosestAllianceTrapPose(Odometry.getInstance().getPose());
        Translation2d offsetTranslation = new Translation2d(distance, closestTrap.getRotation());
        
        return new Pose2d(closestTrap.getTranslation().plus(offsetTranslation), closestTrap.getRotation());
    }

    private boolean shouldSlow() {
        double toTarget = getTargetPose()
            .getTranslation()
            .minus(Odometry.getInstance().getPose().getTranslation())
            .getNorm();

        return toTarget < Units.inchesToMeters(14.0);
    }
    
    @Override
    public void initialize() {
        targetPose = getTargetPose();
    }
    
    @Override
    public void execute() {
        targetPose2d.setPose(targetPose);
        controller.update(targetPose, odometry.getPose());
        
        double rotation = SLMath.clamp(controller.getOutput().omegaRadiansPerSecond, Motion.MAX_ANGULAR_VELOCITY.get());
        if (Math.abs(rotation) < Swerve.ALIGN_OMEGA_DEADBAND.get())
            rotation = 0;
        
        Vector2D speed = new Vector2D(controller.getOutput().vxMetersPerSecond, controller.getOutput().vyMetersPerSecond)
            .clamp(2.0);
        
        if (shouldSlow())
            speed = speed.clamp(0.3);
        
        SmartDashboard.putNumber("Alignment/Translation Target Speed", speed.distance());
        
        swerve.setChassisSpeeds(new ChassisSpeeds(speed.x, speed.y, rotation));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        Field.clearFieldObject(targetPose2d);
    }

}
