package com.stuypulse.robot.subsystems.leds.instructions;

import com.stuypulse.robot.util.SLColor;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSectionNew implements LEDInstruction{
    private SLColor[] colors;
    private int[] lengths;

    
    public void LedSectionNew(SLColor[] colors, int[] lengths){
        this.colors = colors;
        this.lengths = lengths;
    }

    
    public void setLED(AddressableLEDBuffer buffer){
       int colorIndex = 0;
       int lengthIndex = 0;

        if (colors.length != lengths.length){
            throw new IllegalArgumentException("Color and Lengths should be same size");
        }
        
        for (int i = 0; i < buffer.getLength() - 1; i++){
            if (lengths[lengthIndex] < i){
                colorIndex++;
                lengthIndex++;
            }
            buffer.setRGB(lengthIndex, colors[colorIndex].getRed(), colors[colorIndex].getGreen(), colors[colorIndex].getBlue());
        }    
        
        SLColor finalColor =  colors[colors.length - 1];
        buffer.setRGB(buffer.getLength() - 1, finalColor.getRed(), finalColor.getGreen(), finalColor.getBlue());
    }   

    public void sameLength(AddressableLEDBuffer buffer){
        int sectionLength = buffer.getLength() / lengths.length;
        
        int offset = 0;

        for (int i = 0; i < sectionLength; i++) {
            for (int j = 0; j < sectionLength + i % 2; j++) {
                buffer.setRGB(i * sectionLength + offset + j, colors[i].getRed(), colors[i].getGreen(), colors[i].getBlue());
            }
            if (i % 2 == 1) offset += 1;
        }  
    }
}

