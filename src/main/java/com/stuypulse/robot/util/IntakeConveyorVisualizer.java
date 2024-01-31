package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakeConveyorVisualizer {

    private int rollerAngleDirection;
    
    private final double WINDOW_WIDTH = 20;
    private final double WINDOW_HEIGHT = 20;
    private final double WINDOW_X_PADDING = 1;
    private final int LINE_WIDTH = 10;


    private final Mechanism2d intake;

    // Roots
    private MechanismRoot2d rollerRoot1;
    private MechanismRoot2d rollerRoot2;

    private MechanismRoot2d axleRoot1;
    private MechanismRoot2d axleRoot2;
    private MechanismRoot2d axleRoot3;
    private MechanismRoot2d axleRoot4;
    private MechanismRoot2d axleRoot5;
    private MechanismRoot2d axleRoot6;
    

    // Ligaments
    private MechanismLigament2d rollerLigament1;
    private MechanismLigament2d rollerLigament2;

    private MechanismLigament2d axleLigament1;
    private MechanismLigament2d axleLigament2;
    private MechanismLigament2d axleLigament3;
    private MechanismLigament2d axleLigament4;
    private MechanismLigament2d axleLigament5;
    private MechanismLigament2d axleLigament6;

    private MechanismLigament2d chain1;
    private MechanismLigament2d chain2;
    private MechanismLigament2d chain3;
    private MechanismLigament2d chain4;
    private MechanismLigament2d chain5;
    private MechanismLigament2d chain6;



    // colors
    private Color8Bit white = new Color8Bit(255,255,255);
    private Color8Bit blue = new Color8Bit(0, 0, 255);
    private Color8Bit red = new Color8Bit(255, 0, 0);
    private Color8Bit green = new Color8Bit(0, 255, 0);
    private Color8Bit yellow = new Color8Bit(255, 255, 0);


    private MechanismLigament2d getLigament(String name, double length, double angle, Color8Bit color) {
        return new MechanismLigament2d(name, length, angle, LINE_WIDTH, color);
    }

    public IntakeConveyorVisualizer() {
        intake = new Mechanism2d(WINDOW_WIDTH, WINDOW_HEIGHT);
        
        // (roots) 

        // rollers    
        rollerRoot1 = intake.getRoot("roller root 1", 2, 2);
        rollerRoot2 = intake.getRoot("roller root 2", 4, 1);
        
        // axles 
        axleRoot1 = intake.getRoot("axle root 1", 2, 10);
        axleRoot2 = intake.getRoot("axle root 2", 6, 12);
        axleRoot3 = intake.getRoot("axle root 3", 9, 14);
        axleRoot4 = intake.getRoot("axle root 4", 4, 5);
        axleRoot5 = intake.getRoot("axle root 5", 4, 8);
        axleRoot6 = intake.getRoot("axle root 6", 7, 9);

        // (ligaments)

        // rollers 
        rollerLigament1 = new MechanismLigament2d("roller 1", .1, 0, 15, white);
        rollerLigament2 = new MechanismLigament2d("roller 2", .1, 0, 15, white);
        

        // axles 
        axleLigament1 = new MechanismLigament2d("axle 1", .1, 0, 15, white);
        axleLigament2 = new MechanismLigament2d("axle 2", .1, 0, 15, white);
        axleLigament3 = new MechanismLigament2d("axle 3", .1, 0, 15, white);
        axleLigament4 = new MechanismLigament2d("axle 4", .1, 0, 15, white);
        axleLigament5 = new MechanismLigament2d("axle 5", .1, 0, 15, white);
        axleLigament6 = new MechanismLigament2d("axle 6", .1, 0, 15, white);

        // chains
        chain1 = getLigament("chain 1", 8, 90, blue);
        chain2 = getLigament("chain 2", 3, 45, blue);
        chain3 = getLigament("chain 3", 5, 20, blue);
        chain4 = getLigament("chain 4", 4, 90, blue);
        chain5 = getLigament("chain 5", 3, 90, blue);
        chain6 = getLigament("chain 6", 2, 0, blue);
        
        rollerRoot1.append(chain1);
        rollerRoot2.append(chain4);
        axleRoot1.append(chain2);
        axleRoot2.append(chain3);
        axleRoot4.append(chain5);
        axleRoot5.append(chain6);

        rollerRoot1.append(rollerLigament1);
        rollerRoot2.append(rollerLigament2);
        axleRoot1.append(axleLigament1);
        axleRoot2.append(axleLigament2);
        axleRoot3.append(axleLigament3);
        axleRoot4.append(axleLigament4);
        axleRoot5.append(axleLigament5);
        axleRoot6.append(axleLigament6);

        SmartDashboard.putData("Intake", intake);
    }


    // public void update(boolean intakeIR, boolean shooterIR, boolean amperIR) {
    //     if (intakeIR) {
    //         intakeirSensor1.setColor(green);
    //     } else {
    //         intakeirSensor1.setColor(red);
    //     }

    //     if (intakeIR) {
    //         intakeirSensor2.setColor(green);
    //     } else {
    //         intakeirSensor2.setColor(red);
    //     }

    //     if (intakeIR) {
    //         intakeirSensor3.setColor(green);
    //     } else {
    //         intakeirSensor3.setColor(red);
    //     }
    // }


    // update acquire and deacquire accurately later
    private void acquire() {
        rollerLigament1.setAngle(rollerAngleDirection + 1);
        rollerAngleDirection += 1;
    }

    private void deacquire() {
        rollerLigament1.setAngle(rollerAngleDirection - 1);
        rollerAngleDirection -= 1;
    }

}