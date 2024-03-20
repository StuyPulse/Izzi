package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDriveToClimb extends SwerveDriveToPose {
    
    public SwerveDriveToClimb() {
        this(Alignment.TRAP_SETUP_DISTANCE.get());
    }

    public SwerveDriveToClimb(double distance) {
        super(
            () -> {
                Pose2d closestTrap = Field.getClosestAllianceTrapPose(Odometry.getInstance().getPose());
                Translation2d offsetTranslation = new Translation2d(distance, closestTrap.getRotation());
                
                return new Pose2d(closestTrap.getTranslation().plus(offsetTranslation), closestTrap.getRotation());
            }
        );
        
        withTolerance(3.0, 3.0, 3.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
