package com.stuypulse.robot.util;

import java.io.File;
import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Scanner;
import java.util.function.Function;
import java.util.stream.Collectors;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Driver.Drive;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
            for (int i = 0; i < paths.length; i++) {
                if (!PathUtil.findClosestMatch(PathUtil.getPathFileNames(), paths[i].toString()).equals(paths[i].toString())) {
                    DriverStation.reportError("Path " + paths[i] + " not found. Using closest match: " + PathUtil.findClosestMatch(PathUtil.getPathFileNames(), paths[i].toString()), false);
                }             
            }
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
        HashMap<String, Double> hm = new HashMap<>();
        double minValue = Double.MAX_VALUE;
        String minString = "";

        for (int i = 0; i < paths.size(); i++ ) {
            String path = paths.get(i);
            File file = new File(path);
   
            if (path.isEmpty()) {
                System.out.println("Paths are empty");
            }
    
            if (input.isEmpty()) {
                System.out.println("Input is empty");
            } 
    
            Pair<String, Double> tempPair = new Pair<>(path, distance(file.getName(), input));

            for (int z = 0; z < paths.size(); z++){
                hm.put(path, tempPair.getSecond());
            }

            for (int j = 0; j < hm.size(); j++){
                if (Math.min(tempPair.getSecond(), minValue) == tempPair.getSecond()) {
                    minString = tempPair.getFirst();
                }
            }  
        }

        return minString;
    }

    private static double distance(String string1, String string2) {
        Map<Character, Double> frequencyOne = frequency(string1);
        Map<Character, Double> frequencyTwo = frequency(string2);

        ArrayList<Double> list = new ArrayList<>();
        for (char c = 'A'; c <= 'Z'; c++) {
            list.add(Math.pow(frequencyOne.get(c) - frequencyTwo.get(c), 2));
        }
        return Math.sqrt(list.stream().mapToDouble(Double::doubleValue).sum());
    }

    private static Map<Character, Double> frequency(String fileString) {
        Scanner sc = new Scanner(fileString);

        HashMap<Character, Integer> map = new HashMap<>();
        int total = 0;

        while (sc.hasNextLine()) {
            String line = sc.nextLine();
            for (int i = 0; i < line.length(); i++) {
                if (Character.isAlphabetic(line.charAt(i))) {
                    total++;
                    char c = Character.toUpperCase(line.charAt(i));
                    if (map.containsKey(c)) {
                        map.put(c, map.get(c) + 1);
                    } else {
                        map.put(c, 1);
                    }
                }
            }
        }
        sc.close();

        return listFrequency(map, total);
    }

    private static Map<Character, Double> listFrequency(HashMap<Character, Integer> map, int total) {
        Map<Character, Double> frequency = new HashMap<>();
        for (char c = 'A'; c <= 'Z'; c++) {
            if (map.containsKey(c)) {
                frequency.put(c, Math.round((double) map.get(c) / total * 100000) / 100000.0);
            } else {
                frequency.put(c, 0.0);
            }
        }
        return frequency;
    }
}
