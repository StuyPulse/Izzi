/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.leds.instructions;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.util.SLColor;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.util.Arrays;

public class LEDAutonChooser extends LEDSection {
    public enum AutonLEDColors {
        X("DoNothingAuton"),
        OM("1 Piece + Mobility"),
        OC("2 Piece C"),
        OH("2 Piece H"),
        OCB("3 Piece CB"),
        OHG("3 Piece HG"),
        OGHF("4 Piece GHF"),
        OHGF("4 Piece HGF"),
        OCBA("4 Piece CBA"),
        OCBADF(/* "6 Piece CBADF (ND+PF).auto", "6 Piece CBADF (ND).auto",*/ "6 Piece CBADF"),
        M("Mobility.auto");

        public final String autonName;
        public SLColor[] ledColors;

        private AutonLEDColors(String autonName) {
            this.autonName = autonName;
            this.ledColors = autonToLEDSection(name().toCharArray());
        }

        private static SLColor[] autonToLEDSection(char[] pieces) {
            SLColor[] colorArray = new SLColor[10];
            Arrays.fill(colorArray, new SLColor());

            SLColor[] rainbow = new SLColor[] {
                SLColor.RED,
                SLColor.ORANGE,
                SLColor.YELLOW,
                SLColor.LIME,
                SLColor.GREEN,
                SLColor.BLUE,
                SLColor.PURPLE
            };

            int iter = 0;

            for (char ledIndex : pieces) {
                int ascii = (int) ledIndex;
                String pieceId = String.valueOf(ledIndex);
                if (pieceId.equals("O")) {
                    colorArray[0] = SLColor.GREEN;
                } else if (!pieceId.equals("M") && !pieceId.equals("T") && !pieceId.equals("X")) {
                    colorArray[ascii - 64] = rainbow[iter];
                    iter += 1;
                }
            }

            colorArray[9] =  Robot.isBlue() ? SLColor.BLUE : SLColor.RED;
            return colorArray;
        }

        public static AutonLEDColors fromName(String name) {
            return Arrays.stream(values())
                .filter((autonLedColor) -> autonLedColor.autonName.matches(name))
                .findFirst()
                .orElseThrow(
                    () -> new IllegalArgumentException(
                        "No LED configuration for auton with name: "
                            + name
                            + " found"));
        }
    }

    public LEDAutonChooser() {
        super(AutonLEDColors.fromName(RobotContainer.getAutonomousCommandNameStatic()).ledColors);
    }

    @Override
    public void setLED(AddressableLEDBuffer ledBuffer) {
        super.setLED(ledBuffer);
        ledBuffer.setLED(ledBuffer.getLength() - 1, Robot.isBlue() ? SLColor.BLUE : SLColor.RED);
    }
}
