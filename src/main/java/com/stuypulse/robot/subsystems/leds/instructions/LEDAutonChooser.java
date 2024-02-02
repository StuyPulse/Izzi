package com.stuypulse.robot.subsystems.leds.instructions;

import java.util.HashMap;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.util.SLColor;

import edu.wpi.first.wpilibj.DriverStation;

public class LEDAutonChooser {
    public enum AUTONS {
        OM("1 Note + Mobility.auto"),
        OC("2 Note C.auto"),
        OF("2 Note Center F.auto"),
        OG("2 Note Center G.auto"),
        OH("2 Note Center H.auto"),
        ODA("3 Note (D, A).auto"),
        OEA("3 Note (E, A).auto"),
        OGF("3 Note Center (G, F).auto"),
        OHG("3 Note Center (H, G).auto"),
        //("3 Note.auto"),
        OEDA("4 Note (E, D, A).auto"),
        //("4 Note End D.auto"),
        //OES("4 Note End Speaker.auto"),
        //("5 Note.auto"),
        //("6 Note.auto"),
        M("Mobility.auto");

        public final String autonName;

        private AUTONS(String autonName) {
            this.autonName = autonName;
        }

        public String[] parseEnumName() {
            return name().split(" ");
            /*
             * example: OGH.parseEnumName() returns ["O", "G", "H"]
             */
        }

    }

    private HashMap<AUTONS, LEDSection> hashMap;

    public LEDAutonChooser() {
        hashMap = new HashMap<>();
        //get the list of autons
    }


    //TODO: NEWBIES FINISH THIS METHOD and then ill explain how to write the constructor and other stuff
    public LEDSection autonToLEDSection(AUTONS auton) {
        SLColor[] colorArray = new SLColor[10]; 
        /*XXX: order: OABCDEFGHT   O for preloaded note, T for alliance*/
 
        //INSTRUCTIONS FOR THE NEWBIES:
        /*
        use the parseEnumName() method and map the letter to the LEDSection index
        for example if the auton is OGH, then the parseEnumName() method returns ["O", "G", "H"]
        you would map "O" to index 0, "G" to index 7 and "H" to index 8 in the colorArray
        if it is being used, set the color to be green at that index in the LEDSection section
        if its mobility dont light anything
        */
       
        //this gets the alliance color and sets it to the last index of the colorArray (dont touch it, it works as is) 
        colorArray[9] = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? SLColor.BLUE : SLColor.RED);

        return new LEDSection(colorArray); 
    }

    
}
