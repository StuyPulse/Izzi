/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.amper;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Field.NamedTags;
import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AmperToHeight extends InstantCommand {

    public static Command untilDone(double height) {
        return new AmperToHeight(height)
            .andThen(new WaitUntilCommand(() -> Amper.getInstance().isAtTargetHeight(Lift.MAX_HEIGHT_ERROR)));
    }

    private final Amper amper;
    private final double height;

    public AmperToHeight(double height) {
        amper = Amper.getInstance();
        this.height = height;

        addRequirements(amper);
    }

    @Override
    public void initialize() {
        amper.setTargetHeight(height);
    }

    public static boolean isUnderStage() {
        Translation2d[] blueTriangle = new Translation2d[] {
            new Translation2d(Units.inchesToMeters(126.13), Units.inchesToMeters(205.78)), //center 
            new Translation2d(Units.inchesToMeters(218), Units.inchesToMeters(151.68)), //bottom
            new Translation2d(Units.inchesToMeters(218), Units.inchesToMeters(269))        //top
        };

        Translation2d[] redTriangle = new Translation2d[] {
            new Translation2d(Field.LENGTH - blueTriangle[0].getX(), blueTriangle[0].getY()), //center 
            new Translation2d(Field.LENGTH - blueTriangle[1].getX(), blueTriangle[1].getY()), //bottom
            new Translation2d(Field.LENGTH - blueTriangle[2].getX(), blueTriangle[2].getY()), //top
        };

        double[] slopes = new double[3];
        double[] yIntercepts = new double[3];

        // check for alliance side
        if (Robot.isBlue()) { 
            //constructing lines from the triangles to check if the robot is under the stage
            for (int i = 0; i < 3; i++) {
                slopes[i] = (blueTriangle[(i + 1) % 3].getY() - blueTriangle[i].getY()) / (blueTriangle[(i + 1) % 3].getX() - blueTriangle[i].getX());
                yIntercepts[i] = blueTriangle[i].getY() - slopes[i] * blueTriangle[i].getX();
            }
        }
        else {
            for (int i = 0; i < 3; i++) {
                slopes[i] = (redTriangle[(i + 1) % 3].getY() - redTriangle[i].getY()) / (redTriangle[(i + 1) % 3].getX() - redTriangle[i].getX());
                yIntercepts[i] = redTriangle[i].getY() - slopes[i] * redTriangle[i].getX();
            }
        }

        //checking if the robot is under the stage by comparing the robot's position to the lines
        Pose2d robotPose = Odometry.getInstance().getPose();
        for (int i = 0; i < 3; i++) {
            if (robotPose.getTranslation().getY() > slopes[i] * robotPose.getTranslation().getX() + yIntercepts[i] && robotPose.getTranslation().getY() > blueTriangle[1].getY() && robotPose.getTranslation().getY() < blueTriangle[2].getY()) {
                if ((robotPose.getTranslation().getX() > blueTriangle[0].getX() 
                  && robotPose.getTranslation().getX() < blueTriangle[2].getX()) ||
                    (robotPose.getTranslation().getX() < redTriangle[0].getX() 
                  && robotPose.getTranslation().getX() > redTriangle[2].getX())) {
                    return true;
                }
            }
        }
        return false;
    }
}
