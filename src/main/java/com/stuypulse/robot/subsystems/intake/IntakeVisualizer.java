package com.stuypulse.robot.subsystems.intake;

import static com.stuypulse.robot.constants.Settings.Intake.*;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakeVisualizer {
    private final double WINDOW_WIDTH = 6;
    private final double WINDOW_HEIGHT = 15;
    private final double WINDOW_X_PADDING = 1;
    private final int LINE_WIDTH = 8;



    private final Mechanism2d intake;

    // ligaments
   
    private MechanismLigament2d upper1;
    private MechanismLigament2d lower1;

    private MechanismLigament2d upper2;
    private MechanismLigament2d lower2;

    private MechanismLigament2d upper3;
    private MechanismLigament2d lower3;
   
   // 1 is upper, 2 is lower
    private MechanismLigament2d middle1;
    private MechanismLigament2d middle2;
    
   
     // roots
    private MechanismRoot2d root_upper1;
    private MechanismRoot2d root_upper2;
    private MechanismRoot2d root_upper3;
    private MechanismRoot2d root_lower1;
    private MechanismRoot2d root_lower2;
    private MechanismRoot2d root_lower3;
    private MechanismRoot2d root_middle;

// colors
    private Color8Bit white = new Color8Bit(255,255,255);
    private Color8Bit blue = new Color8Bit(0, 0, 255);
    private Color8Bit red = new Color8Bit(255, 0, 0);


    private MechanismLigament2d getLigament(String name, double length, double angle, Color8Bit color) {
        return new MechanismLigament2d(name, length, angle, LINE_WIDTH, color);
    }


public IntakeVisualizer(){

  intake = new Mechanism2d(WINDOW_WIDTH, WINDOW_HEIGHT, null);
  
// uppper
    root_upper1 = intake.getRoot("one", 1, 1);
    root_upper2 = intake.getRoot("two", 1, 1);
    root_upper3 = intake.getRoot("three", 1, 1);
   
// lower
    root_lower1 = intake.getRoot("one", 1, 1);
    root_lower2 = intake.getRoot("two", 1, 1);
    root_lower3 = intake.getRoot("three", 1, 1);
   
// middle
    root_middle = intake.getRoot("one", 1, 1);

// ligaments
    upper1 = getLigament("Upper ligament one", 0, 0, blue);
    lower1 = getLigament("Lower ligament one", 0, 0, blue);
    upper2 = getLigament("Upper ligament two", 0, 0, red);
    lower2 = getLigament("lower ligament two", 0, 0, red);
    upper3 = getLigament("lower ligament three", 0, 0, white);
    lower3 = getLigament("lower ligament three", 0, 0, white);
    middle1 = getLigament("middle ligament one", 0, 0, white);
    middle2 = getLigament("middle ligament two", 0, 0, white);

    root_upper1.append(upper1);
    root_lower1.append(lower1);
    root_upper2.append(upper2);
    root_lower2.append(lower2);
    root_upper3.append(upper3);
    root_lower3.append(lower3);
    root_middle.append(middle1);
    root_middle.append(middle2);

    SmartDashboard.putData("Intake", intake);
    



}


 
    
}