/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ClimberVisualizer {
    private final double WINDOW_WIDTH = 6;
    private final double WINDOW_HEIGHT = 15;
    private final double WINDOW_X_PADDING = 1;

    private final double OUTER_STAGE_HEIGHT = 6;

    private final int LINE_WIDTH = 8;

    private final Mechanism2d climber;

    // ligaments
    private MechanismLigament2d outerLeftLigament;
    private MechanismLigament2d outerRightLigament;

    private MechanismLigament2d topLigament;

    private MechanismLigament2d leftLigament;
    private MechanismLigament2d rightLigament;

    // roots
    private MechanismRoot2d topRoot;

    private MechanismRoot2d leftRoot;
    private MechanismRoot2d rightRoot;

    private MechanismRoot2d rightBottomRoot;
    private MechanismRoot2d leftBottomRoot;

    private double leftRootX;
    private double rightRootX;

    // colors
    private Color8Bit blue = new Color8Bit(0, 0, 255);
    private Color8Bit red = new Color8Bit(255, 0, 0);

    private MechanismLigament2d getLigament(String name, double length, double angle, Color8Bit color) {
        return new MechanismLigament2d(name, length, angle, LINE_WIDTH, color);
    }

    public ClimberVisualizer() {
        climber = new Mechanism2d(WINDOW_WIDTH, WINDOW_HEIGHT);

        leftRootX = WINDOW_X_PADDING;
        rightRootX = WINDOW_WIDTH - WINDOW_X_PADDING;

        // root nodes

        // outer shell
        leftRoot = climber.getRoot("left root", leftRootX, 0);
        rightRoot = climber.getRoot("right root", rightRootX, 0);
        topRoot = climber.getRoot("top root", leftRootX, OUTER_STAGE_HEIGHT);

        // inner shell
        leftBottomRoot = climber.getRoot("left bottom root", leftRootX, 0);
        rightBottomRoot = climber.getRoot("right bottom root", rightRootX, 0);

        // ligaments

        // outer shell
        topLigament = getLigament("top ligament", WINDOW_WIDTH - 2 * WINDOW_X_PADDING, 0, blue);
        outerRightLigament = getLigament("outer right ligament", OUTER_STAGE_HEIGHT, 90, blue);
        outerLeftLigament = getLigament("outer left ligament", OUTER_STAGE_HEIGHT, 90, blue);

        // inner shell
        leftLigament = new MechanismLigament2d("climber left ligament ", OUTER_STAGE_HEIGHT - 5, 90, 12, red);
        rightLigament = new MechanismLigament2d("climber right ligament ", OUTER_STAGE_HEIGHT - 5, 90, 12, red);

        // outer shell
        leftRoot.append(outerLeftLigament);
        rightRoot.append(outerRightLigament);
        topRoot.append(topLigament);

        // inner shell
        leftBottomRoot.append(leftLigament);
        rightBottomRoot.append(rightLigament);

        SmartDashboard.putData("Visualizers/Climber", climber);
    }

    public void setLeftHeight(double newHeight) {
        double percentDone = newHeight / Settings.Climber.MAX_HEIGHT;

        double stageBottomY = OUTER_STAGE_HEIGHT * percentDone;

        leftBottomRoot.setPosition(leftRootX, stageBottomY);
    }

    public void setRightHeight(double newHeight) {
        double percentDone = newHeight / Settings.Climber.MAX_HEIGHT;

        double stageBottomY = OUTER_STAGE_HEIGHT * percentDone;

        rightBottomRoot.setPosition(leftRootX, stageBottomY);
    }
}
