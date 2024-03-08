package com.stuypulse.robot.util;

import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class PathUtil {

    public static class AutonConfig {
    
        private final String name;
        private final Function<PathPlannerPath[], Command> auton;
        private final String[] paths;

        public AutonConfig(String name, Function<PathPlannerPath[], Command> auton, String... paths) {
            this.name = name;
            this.auton = auton;
            this.paths = paths;
        }
        
        public AutonConfig registerBlue(SendableChooser<Command> chooser) {
            chooser.addOption("Blue " + name, auton.apply(loadPaths(paths)));
            return this;
        }

        public AutonConfig registerRed(SendableChooser<Command> chooser) {
            chooser.addOption("Red " + name, auton.apply(loadPathsRed(paths)));
            return this;
        }
                
        public AutonConfig registerDefaultBlue(SendableChooser<Command> chooser) {
            chooser.setDefaultOption("Blue " + name, auton.apply(loadPaths(paths)));
            return this;
        }

        public AutonConfig registerDefaultRed(SendableChooser<Command> chooser) {
            chooser.setDefaultOption("Red " + name, auton.apply(loadPathsRed(paths)));
            return this;
        }

    }

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

    public static List<String> getPathFileNames() {
        //  ../../../../../deploy/pathplanner/paths

        Path path = Paths.get("").toAbsolutePath().resolve("src/main/deploy/pathplanner/paths");
        ArrayList<String> fileList = new ArrayList<String>();
        try (DirectoryStream<Path> stream = Files.newDirectoryStream(path, "*.path")) {
            for (Path file: stream){
                fileList.add(file.getFileName().toString().replaceFirst(".path",""));
            }
        } catch (IOException error) {
            System.err.println(error);
        }
        return fileList;
    }
    
    public static String findClosestMatch(List<String> paths, String input) {
        HashMap<String, Integer> hm = new HashMap<>();
        int minValue = Integer.MAX_VALUE;
        String minString = "";

        for (int i = 0; i < paths.size(); i++ ) {
            String temp = paths.get(i);
            if (temp.isEmpty()) {
                System.out.println("Paths are empty");
            }
    
            if (input.isEmpty()) {
                System.out.println("Input is empty");
            } 
    
            int substitution = calculate(temp.substring(1), input.substring(1)) 
             + costOfSubstitution(temp.charAt(0), input.charAt(0));
            int insertion = calculate(temp, input.substring(1)) + 1;
            int deletion = calculate(temp.substring(1), input) + 1;
                        
            Pair<String, Integer> tempPair = new Pair<String,Integer>(temp, min(deletion, insertion, substitution));

            for (int z = 0; z < paths.size(); z++){
                hm.put(temp, min(deletion, insertion, substitution));
            }

            for (int j = 0; j < hm.size(); j++){
                if (hm.get(tempPair.getFirst()) < minValue) {
                    minString = tempPair.getFirst();
                }
            }  
        }

        return minString;
        
    }

    public static int costOfSubstitution(char a, char b) {
        return a == b ? 0 : 1;
    }

    public static int min(int... numbers) {
        return Arrays.stream(numbers)
          .min().orElse(Integer.MAX_VALUE);
    }

    static int calculate(String x, String y) {
        if (x.isEmpty()) {
            return y.length();
        }

        if (y.isEmpty()) {
            return x.length();
        } 

        int substitution = calculate(x.substring(1), y.substring(1)) 
         + costOfSubstitution(x.charAt(0), y.charAt(0));
        int insertion = calculate(x, y.substring(1)) + 1;
        int deletion = calculate(x.substring(1), y) + 1;

        return min(substitution, insertion, deletion);
    }
    // public static void main(String[] args) { why did you put that there
        
    // }
}
