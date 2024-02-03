package com.stuypulse.robot.subsystems.leds.instructions;

import java.util.EnumMap;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.util.SLColor;

import edu.wpi.first.wpilibj.DriverStation;

public class LEDAutonChooser extends LEDSection {
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

        public char[] parseEnumName() {
            return name().toCharArray();
        }

    }

    private static EnumMap<AUTONS, SLColor[]> enumMap = new EnumMap<>(AUTONS.class);

    static {
        for (AUTONS auton : AUTONS.values()){
            enumMap.put(auton, autonToLEDSection(auton));
            
        }
    }

    public LEDAutonChooser(PathPlannerAuto auton) {
        super(enumMap.get(AUTONS.valueOf(auton.getName())));
    }
    
    private static SLColor[] autonToLEDSection(AUTONS auton) {
        SLColor[] colorArray = new SLColor[10];
        
        SLColor[] rainbow = new SLColor[] {SLColor.RED, SLColor.RED_ORANGE, SLColor.ORANGE, SLColor.YELLOW, SLColor.LIME, SLColor.GREEN, SLColor.BLUE, SLColor.PURPLE};

        int iter = 0;

        for (char ledIndex : auton.parseEnumName()){
            int ascii = (int) ledIndex;
            String pieceId = String.valueOf(ledIndex);
            if (pieceId.equals("O")) {
                colorArray[0] = SLColor.GREEN;
            }
            else if (!pieceId.equals("M") && !pieceId.equals("T") ) {  
                colorArray[ascii - 64] = rainbow[iter]; 
                iter += 1;
            }
        }
        
        for (int i = 1; i < colorArray.length - 1; i++ ){
            if (colorArray[i] == null) colorArray[i] = new SLColor();
        }

        colorArray[9] = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? SLColor.BLUE : SLColor.RED);
        return colorArray; 
    }
}