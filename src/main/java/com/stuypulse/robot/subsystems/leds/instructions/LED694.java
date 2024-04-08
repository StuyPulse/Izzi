/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.leds.instructions;

import java.util.ArrayList;

import com.stuypulse.robot.util.SLColor;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED694 implements LEDInstruction {

    private final double pulsingTime;

    public LED694(double pulsingTime) {
        this.pulsingTime = pulsingTime;
    }

    public LED694() {
        this(0.03);
    }

    private SLColor[] colorMap = {SLColor.RED, SLColor.WHITE};
    private ArrayList<Integer> colorArray = initColors();
    private StopWatch stopWatch = new StopWatch();

    @Override
    public void setLED(AddressableLEDBuffer ledBuffer) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, colorMap[colorArray.get(i)]);
        }

        if (pulsingTime < stopWatch.getTime()){
            int firstColor = colorArray.get(0);
            for (int i = 1; i < colorArray.size(); i++){
                colorArray.set(i - 1, colorArray.get(i));
            }
            colorArray.set(colorArray.size() - 1, firstColor);

            stopWatch.reset();
        }
    }


    private static ArrayList<Integer> initColors() {
        ArrayList<Integer> colors = new ArrayList<>();
        for (int i = 0; i < 6; i++) colors.add(0);
        for (int i = 0; i < 6; i++) colors.add(1);
        for (int i = 0; i < 9; i++) colors.add(0);
        for (int i = 0; i < 9; i++) colors.add(1);
        for (int i = 0; i < 4; i++) colors.add(0);
        for (int i = 0; i < 4; i++) colors.add(1);
        return colors;
    }
    
}
