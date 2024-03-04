package com.stuypulse.robot.util;

import java.util.List;
import java.util.stream.Collectors;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PathUtil {
    public static PathPlannerPath[] loadPathsRed(String... names) {
        PathPlannerPath[] output = new PathPlannerPath[names.length];
        for (int i = 0; i < names.length; i++) {
            output[i] = loadRed(names[i]);
        }
        return output;
    }

    public static PathPlannerPath[] loadPaths(String... names) {
        PathPlannerPath[] output = new PathPlannerPath[names.length];
        for (int i = 0; i < names.length; i++) {
            output[i] = load(names[i]);
        }
        return output;
    }

    public static PathPlannerPath load(String name) {
        return PathPlannerPath.fromPathFile(name);
    }
    
    public static PathPlannerPath loadRed(String name) {
        return flipPath(PathPlannerPath.fromPathFile(name));
    }

    public static Translation2d flipFieldTranslation(Translation2d pose) {
        return new Translation2d(pose.getX(), Field.WIDTH - pose.getY());
    }

    public static Rotation2d flipFieldRotation(Rotation2d rotation) {
        return rotation.times(-1);
    }

    public static Pose2d flipFieldPose(Pose2d pose) {
        return new Pose2d(
            flipFieldTranslation(pose.getTranslation()),
            flipFieldRotation(pose.getRotation()));
    }

    public static PathPoint flipPathPoint(PathPoint point) {
        return new PathPoint(
            flipFieldTranslation(point.position), 
            point.rotationTarget == null ? null : new RotationTarget(
                point.rotationTarget.getPosition(),
                flipFieldRotation(point.rotationTarget.getTarget())),
            point.constraints
        );
    }

    public static PathPlannerPath flipPath(PathPlannerPath path) {
        List<PathPoint> newPathPoints = path.getAllPathPoints()
            .stream().map(PathUtil::flipPathPoint)
            .collect(Collectors.toList());
        
        GoalEndState newEndState =
            new GoalEndState(
                path.getGoalEndState().getVelocity(),
                flipFieldRotation(path.getGoalEndState().getRotation()),
                path.getGoalEndState().shouldRotateFast());

        return PathPlannerPath.fromPathPoints(
            newPathPoints,
            path.getGlobalConstraints(),
            newEndState
        );
    }
}
