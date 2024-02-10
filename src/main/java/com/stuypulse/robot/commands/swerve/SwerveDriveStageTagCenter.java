package com.stuypulse.robot.commands.swerve;

import java.util.ArrayList;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.util.vision.Fiducial;
import com.stuypulse.robot.util.vision.VisionData;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class SwerveDriveStageTagCenter extends SwerveDriveToPose {

    public SwerveDriveStageTagCenter() {
        super(getTargetPose());
    }

    public static int pickAprilTag() {
        boolean isBlue = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

        double minDist = Double.MAX_VALUE;
        int minFid = -1;

        ArrayList<VisionData> outputs = AprilTagVision.getInstance().getOutputs();
        for(VisionData output : outputs) {
            int fids[] = output.getFids();
            for(int fid : fids) {
                minDist = Math.min(output.getDistanceToFiducial(fid), minDist);
                if (minDist == output.getDistanceToFiducial(fid)) {
                    minFid = fid;
                }
            }
        }

        if (isBlue && (minFid == 15 || minFid == 16 || minFid == 14)) {
            return minFid;
        } 
        else if (minFid == 11 || minFid == 12 || minFid == 13) {
            return minFid;
        }
        else {
            return -1;
        }
    }

    public static Pose2d getTargetPose() {
        Fiducial targetPosition = Field.getFiducial(pickAprilTag());

        Translation2d targetTrans = new Translation2d(targetPosition.getLocation().getX(), targetPosition.getLocation().getY());

        double targetAngle = Math.atan(
            (Odometry.getInstance().getPose().getY() - targetPosition.getLocation().getY())/ 
            (Odometry.getInstance().getPose().getX() - targetPosition.getLocation().getX()));

        return new Pose2d(targetTrans, new Rotation2d(targetAngle));
    }    
}
