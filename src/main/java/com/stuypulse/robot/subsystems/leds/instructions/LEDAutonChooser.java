/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.leds.instructions;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Settings.LED;
import com.stuypulse.robot.util.SLColor;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Arrays;
        
public class LEDAutonChooser extends LEDSection {
    public enum AutonLEDColors {
        X("DoNothingAuton"),
        OC("TwoPieceC"),
        OH("TwoPieceH"),
        OG("TwoPieceG"),
        OCB("ThreePieceCB"),
        OHG("ThreePieceHG"),
        OGH("ThreePieceGH"),
        OHGF("FourPieceHGF"),
        OCBA("FourPieceCBA"),
        OCBAD("FivePieceCBAD"),
        OCBAE("FivePieceCBAE"),
        OCBADE("SixPieceCBADE"),
        M("Mobility");

        public final String autonName;
        public SLColor[] ledColors;

        private AutonLEDColors(String autonName) {
            this.autonName = autonName;
            this.ledColors = autonToLEDSection(name().toCharArray());
        }

        private static SLColor[] autonToLEDSection(char[] pieces) {
            SLColor[] colorArray = new SLColor[15];
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

            colorArray[9] = SLColor.WHITE; 
            for (int i = 10; i < LED.LED_LENGTH; i++) {
                colorArray[i] = (Robot.isBlue() ? SLColor.BLUE : SLColor.RED);
            }
            
            return colorArray;
        }

        public static AutonLEDColors fromName(String name) {
            return Arrays.stream(values())
                .filter((autonLedColor) -> autonLedColor.autonName.equals(name))
                .findFirst()
                .orElseGet(() -> {
                    DriverStation.reportWarning("AutonLEDColors.fromName: " + name + " not found", false);
                    return X;
                });
        }
    }

    public LEDAutonChooser() {
        super(true, AutonLEDColors.fromName(RobotContainer.getAutonomousCommandNameStatic()).ledColors);
    }

    @Override
    public void setLED(AddressableLEDBuffer ledBuffer) {
        super.setLED(ledBuffer);
    }
}
