/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakeVisualizer {

    private double intakeAngle = 0.0;
    private double gandalfAngle = 0.0;
    private double shooterFeederAngle = 0.0;

    private final double WINDOW_WIDTH = 20;
    private final double WINDOW_HEIGHT = 20;
    private final int LINE_WIDTH = 10;

    private final Mechanism2d intake;

    // ligaments

    private MechanismLigament2d upper1;
    private MechanismLigament2d lower1;

    private MechanismLigament2d upper2;
    private MechanismLigament2d lower2;

    private MechanismLigament2d upper3;
    private MechanismLigament2d lower3;

    // irSensor
    private MechanismLigament2d intakeIRSensor;
    private MechanismLigament2d ampIRSensor;

    // far Right Ligaments
    private MechanismLigament2d rightLigamentTop;
    private MechanismLigament2d rightLigamentBottom;

    // rolling ligaments
    private MechanismLigament2d rollingLeft;
    private MechanismLigament2d rollingRight;
    private MechanismLigament2d rollingMidLeft;
    private MechanismLigament2d rollingMidRight;
    private MechanismLigament2d rollingShooterTop;
    private MechanismLigament2d rollingShooterMid;
    private MechanismLigament2d rollingConveyorBot;
    private MechanismLigament2d rollingGandalf;

    // roots
    private MechanismRoot2d root_upper1;
    private MechanismRoot2d root_upper2;
    private MechanismRoot2d root_upper3;
    private MechanismRoot2d root_lower1;
    private MechanismRoot2d root_lower2;
    private MechanismRoot2d root_lower3;
    private MechanismRoot2d rootIntakeIR;
    private MechanismRoot2d rootAmpIR;
    private MechanismRoot2d rightRootTop;
    private MechanismRoot2d rightRootBottom;
    private MechanismRoot2d rollingRootMidLeft;
    private MechanismRoot2d rollingRootMidRight;
    private MechanismRoot2d rollingRootShooterTop;
    private MechanismRoot2d rollingRootShooterMid;
    private MechanismRoot2d rollingRootConveyorBot;
    private MechanismRoot2d rollingRootGandalf;

    // colors
    private final Color8Bit WHITE = SLColor.WHITE.getColor8Bit();
    private final Color8Bit BLUE = SLColor.BLUE.getColor8Bit();
    private final Color8Bit RED = SLColor.RED.getColor8Bit();
    private final Color8Bit GREEN = SLColor.GREEN.getColor8Bit();

    
    private MechanismLigament2d getLigament(String name, double length, double angle, Color8Bit color) {
        return new MechanismLigament2d(name, length, angle, LINE_WIDTH, color);
    }

    public IntakeVisualizer() {
        intake = new Mechanism2d(WINDOW_WIDTH, WINDOW_HEIGHT);

        // uppper roots
        root_upper1 = intake.getRoot("One Upper", 0, 0);
        root_upper2 = intake.getRoot("Two Upper", 0, 4);
        root_upper3 = intake.getRoot("Three Upper", 2, 6.2);

        // lower roots
        root_lower1 = intake.getRoot("One Lower", 2, 0);
        root_lower2 = intake.getRoot("Two Lower", 2, 3);
        root_lower3 = intake.getRoot("Three Lower", 3.3, 4.5);

        rootIntakeIR = intake.getRoot("Intake Sensor", 1.5, 4.6);
        rootAmpIR = intake.getRoot("Shooter Sensor", 7, 7.2);

        rollingRootMidLeft = intake.getRoot("Rolling Root Middle Left", 2, 6.2); //find accurate values later
        rollingRootMidRight = intake.getRoot("Rolling Root Middle Right", 3.3, 4.5); //find accurate values later

        // shooter roots
        rollingRootShooterTop = intake.getRoot("Rolling Shooter Top", 9.7, 6.4);
        rollingRootShooterMid = intake.getRoot("Rolling Shooter Mid", 9.7, 5);
        rollingRootConveyorBot = intake.getRoot("Rolling Conveyor Bot", 8.2, 3.6);
        rollingRootGandalf = intake.getRoot("Rolling Gandalf", 7, 5.65);

        // right (separation) roots
        rightRootTop = intake.getRoot("Right Root Top", 7, 5.7);
        rightRootBottom = intake.getRoot("Right Root Bottom", 7, 5.7);

        // ligaments
        upper1 = getLigament("Upper Ligament 1", 4, 90, BLUE);
        upper2 = getLigament("Upper Ligament 2", 3, 45, RED);
        upper3 = getLigament("Upper Ligament 3", 7, 20, WHITE);
        lower1 = getLigament("Lower Ligament 1", 3, 90, BLUE);
        lower2 = getLigament("Lower Ligament 2", 2, 45, RED);
        lower3 = getLigament("Lower Ligament 3", 5, -10, WHITE);

        rollingMidLeft = getLigament("Rolling Ligament Middle Left", .5, 0, GREEN);
        rollingMidRight = getLigament("Rolling Ligament Middle Right", .5, 0, GREEN);

        rollingLeft = getLigament("Left Roller", .5, 0, GREEN);
        rollingRight = getLigament("Right Roller", .5, 0, GREEN);

        rollingShooterTop = getLigament("Top Shooter Roller", .5, 0, GREEN);
        rollingShooterMid = getLigament("Middle Shooter Roller", .5, 0, GREEN);
        rollingConveyorBot = getLigament("Bottom Conveyor Roller", .5, 0, GREEN);

        rollingGandalf = getLigament("rolling Gandalf", .5, 0, GREEN);

        intakeIRSensor = getLigament("Intake Sensor", 1, 0, RED);
        ampIRSensor = getLigament("Amp Sensor", 1, 0, RED);

        rightLigamentTop = getLigament("Right Top Ligament", 3, 15, BLUE);
        rightLigamentBottom = getLigament("Right Bottom Ligament", 3, -15, BLUE);

        root_upper1.append(upper1);
        root_lower1.append(lower1);
        root_upper2.append(upper2);
        root_lower2.append(lower2);
        root_upper3.append(upper3);
        root_lower3.append(lower3);
        rootIntakeIR.append(intakeIRSensor);
        rootAmpIR.append(ampIRSensor);
        rightRootTop.append(rightLigamentTop);
        rightRootBottom.append(rightLigamentBottom);
        root_upper1.append(rollingLeft);
        root_lower1.append(rollingRight);
        rollingRootMidLeft.append(rollingMidLeft);
        rollingRootMidRight.append(rollingMidRight);
        rollingRootShooterTop.append(rollingShooterTop);
        rollingRootShooterMid.append(rollingShooterMid);
        rollingRootConveyorBot.append(rollingConveyorBot);
        rollingRootGandalf.append(rollingGandalf);

        SmartDashboard.putData("Visualizers/Intake", intake);
    }

    public void update(
            boolean intake_IR,
            boolean amp_IR,
            double intakeSpeed,
            double conveyorGandalfSpeed,
            double shooterFeederSpeed) {
        if (intake_IR) {
            intakeIRSensor.setColor(GREEN);
        } else {
            intakeIRSensor.setColor(RED);
        }

        if (amp_IR) {
            ampIRSensor.setColor(GREEN);
        } else {
            ampIRSensor.setColor(RED);
        }

        intakeAngle += 39 * intakeSpeed;
        rollingLeft.setAngle(intakeAngle);
        rollingRight.setAngle(-intakeAngle);
        rollingMidLeft.setAngle(intakeAngle);
        rollingMidRight.setAngle(-intakeAngle);
        // rizz
        gandalfAngle += 39 * conveyorGandalfSpeed;
        rollingGandalf.setAngle(-gandalfAngle);
        rollingConveyorBot.setAngle(gandalfAngle);

        shooterFeederAngle += 39 * shooterFeederSpeed;
        rollingShooterTop.setAngle(-shooterFeederAngle);
        rollingShooterMid.setAngle(shooterFeederAngle);
    }
}
