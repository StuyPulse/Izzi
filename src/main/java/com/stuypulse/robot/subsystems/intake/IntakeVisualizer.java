package com.stuypulse.robot.subsystems.intake;

import static com.stuypulse.robot.constants.Settings.Intake.*;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class IntakeVisualizer {
    private final double WINDOW_WIDTH = 20;
    private final double WINDOW_HEIGHT = 20;
    private final double WINDOW_X_PADDING = 1;
    private final int LINE_WIDTH = 10;


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

    //irSensor
    private MechanismLigament2d intakeIRSensor;
    private MechanismLigament2d shooterIRSensor;
    private MechanismLigament2d ampIRSensor;

     // roots
    private MechanismRoot2d root_upper1;
    private MechanismRoot2d root_upper2;
    private MechanismRoot2d root_upper3;
    private MechanismRoot2d root_lower1;
    private MechanismRoot2d root_lower2;
    private MechanismRoot2d root_lower3;
    private MechanismRoot2d root_middle;
    private MechanismRoot2d root_ir1;
    private MechanismRoot2d root_ir2;
    private MechanismRoot2d root_ir3;



// colors
    private Color8Bit white = new Color8Bit(255,255,255);
    private Color8Bit blue = new Color8Bit(0, 0, 255);
    private Color8Bit red = new Color8Bit(255, 0, 0);
    private Color8Bit green = new Color8Bit();


    private MechanismLigament2d getLigament(String name, double length, double angle, Color8Bit color) {
        return new MechanismLigament2d(name, length, angle, LINE_WIDTH, color);
    }

    public IntakeVisualizer() {
        intake = new Mechanism2d(WINDOW_WIDTH, WINDOW_HEIGHT);
    
        // uppper
        root_upper1 = intake.getRoot("one upper", 0, 0);
        root_upper2 = intake.getRoot("two upper", 0, 4);
        root_upper3 = intake.getRoot("three upper", 2, 6.2);
    
        // lower
        root_lower1 = intake.getRoot("one lower", 2, 0);
        root_lower2 = intake.getRoot("two lower", 2, 3);
        root_lower3 = intake.getRoot("three lower", 3.3, 4.5);
    
        // middle
        root_middle = intake.getRoot("one", 1, 1);

        root_ir1 = intake.getRoot("sensor one", 5, 5);
        root_ir2 = intake.getRoot("sensor two", 5, 5); 
        root_ir3 = intake.getRoot("sensor three", 3, 3);
    

        // ligaments
        upper1 = getLigament("Upper ligament one", 4, 90, blue);
        upper2 = getLigament("Upper ligament two", 3, 45, red);
        upper3 = getLigament("lower ligament three", 5, 20, white);
        lower1 = getLigament("Lower ligament one", 3, 90, blue);
        lower2 = getLigament("lower ligament two", 2, 45, red);
        lower3 = getLigament("lower ligament three", 3, -10, white);
        middle1 = getLigament("middle ligament one", 3, 0, green);
        middle2 = getLigament("middle ligament two", 2, 0, green);
        intakeIRSensor = getLigament("intake ir sensor 1", 1, 0, red);
        shooterIRSensor = getLigament("intake ir sensor 2", 1, 0, red);
        ampIRSensor = getLigament("intake ir sensor 3", 1, 0, red);

        root_upper1.append(upper1); 
        root_lower1.append(lower1);
        root_upper2.append(upper2);
        root_lower2.append(lower2);
        root_upper3.append(upper3);
        root_lower3.append(lower3);
        root_middle.append(middle1);
        root_middle.append(middle2);

        root_ir1.append(intakeIRSensor);
        root_ir2.append(shooterIRSensor);
        root_ir3.append(ampIRSensor);


        SmartDashboard.putData("Intake", intake);
    }

    public void update(boolean intakeIR, boolean shooterIR, boolean amperIR) {
        if (intakeIR) {
            intakeIRSensor.setColor(green);
        } else {
            intakeIRSensor.setColor(red);
        }

        if (shooterIR) {
            shooterIRSensor.setColor(green);
        } else {
            shooterIRSensor.setColor(red);
        }

        if (amperIR) {
            ampIRSensor.setColor(green);
        } else {
            ampIRSensor.setColor(red);
        }
    }

    }