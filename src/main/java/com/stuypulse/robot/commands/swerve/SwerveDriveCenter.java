package com.stuypulse.robot.commands.swerve;

import java.util.ArrayList;

import org.photonvision.estimation.VisionEstimation;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.util.vision.Fiducial;
import com.stuypulse.robot.util.vision.VisionData;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveCenter extends InstantCommand {

    private final Odometry odometry;
    private final HolonomicController controller;
    private SwerveDrive swerve;
    // private Pose2d targetPose;

    private boolean done;

    public SwerveDriveCenter() {
        controller = new HolonomicController(
            new PIDController(),
            new PIDController(),
            new AnglePIDController()
        );

        this.swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();

        addRequirements(swerve);
    }

    public int pickAprilTag() {
        boolean isBlue = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

        double minDist = Double.MAX_VALUE;
        int minFid = -1;

        ArrayList<VisionData> outputs = AprilTagVision.getInstance().getOutputs();
        for(VisionData output : outputs) {
            int fids[] = output.getFids();
            for(int fid : fids) {
                double dist = output.getDistanceToFiducial(fid);
                if(dist < minDist) {
                    minDist = dist;
                    minFid = fid;
                }
            }
        }

        if (isBlue && (minFid == 15 || minFid == 16 || minFid == 14)) {
            return minFid;
        } else if (minFid == 11 || minFid == 12 || minFid == 13) {
            return minFid;
        }
        return -1;
    }

    public Pose2d getTargetPose() {
        Fiducial targetPosition = Field.getFiducial(pickAprilTag());
        double targetPosX = targetPosition.getLocation().getX();
        double targetPosY = targetPosition.getLocation().getY();
        Translation2d targetTrans = new Translation2d(targetPosX, targetPosY);


        double targetAngle = Math.atan((odometry.getPose().getY() - targetPosition.getLocation().getY()) / (odometry.getPose().getX() - targetPosition.getLocation().getX()));
        Rotation2d realTargetAngle = new Rotation2d(targetAngle);

        return new Pose2d(targetTrans, realTargetAngle);
    }

    @Override
    public void execute() {
        new SwerveDriveToPose(getTargetPose());
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {

    }
    
}
