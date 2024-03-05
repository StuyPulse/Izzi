package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;



public class IntakeVisualizer  {

    private final double WINDOW_WIDTH = 20;
    private final double WINDOW_HEIGHT = 20;
    private final int LINE_WIDTH = 10;

    private double intakeAngle;
    private double conveyorAngle;
    private double shooterAngle;

    private final Mechanism2d intake;

    /***ligaments***/

    private MechanismLigament2d intakeBottomRight;
    private MechanismLigament2d intakeBottomLeft;
    
    private MechanismLigament2d intakeTopRight;
    private MechanismLigament2d intakeTopLeft;

    private MechanismLigament2d conveyorBottomRight;
    private MechanismLigament2d conveyorBottomLeft;
    
    private MechanismLigament2d conveyorTopRight;
    private MechanismLigament2d conveyorTopLeft;
    
    private MechanismLigament2d shooterBottomRight;
    private MechanismLigament2d shooterBottomLeft;

    private MechanismLigament2d shooterTopRight;
    private MechanismLigament2d shooterTopLeft;

    private MechanismLigament2d rollerBottomLeft;
    private MechanismLigament2d rollerBottomMiddle;
    private MechanismLigament2d rollerBottomRight;

    private MechanismLigament2d rollerTopLeft;
    private MechanismLigament2d rollerTopMiddle;
    private MechanismLigament2d rollerTopRight;

    /***sensor***/
    
    private MechanismLigament2d conveyorIRSensor;
    
    // roots

    private MechanismRoot2d root_intakeBottomRight;
    private MechanismRoot2d root_intakeBottomLeft;
    
    private MechanismRoot2d root_intakeTopRight;
    private MechanismRoot2d root_intakeTopLeft;

    private MechanismRoot2d root_conveyorBottomRight;
    private MechanismRoot2d root_conveyorBottomLeft;

    private MechanismRoot2d root_conveyorTopRight;
    private MechanismRoot2d root_conveyorTopLeft;

    private MechanismRoot2d root_shooterBottomRight;
    private MechanismRoot2d root_shooterBottomLeft;

    private MechanismRoot2d root_shooterTopRight;
    private MechanismRoot2d root_shooterTopLeft;

    private MechanismRoot2d root_conveyorIRSensor;

    private MechanismRoot2d root_rollerBottomLeft;
    private MechanismRoot2d root_rollerBottomMiddle;
    private MechanismRoot2d root_rollerBottomRight;

    private MechanismRoot2d root_rollerTopLeft;
    private MechanismRoot2d root_rollerTopMiddle;
    private MechanismRoot2d root_rollerTopRight;


    //colors 

    private Color8Bit green = new Color8Bit(0,255,0);
    private Color8Bit red = new Color8Bit(255,0,0); 


    public IntakeVisualizer() {
        intake = new Mechanism2d(WINDOW_WIDTH, WINDOW_HEIGHT);

        // initializing ligaments
        intakeBottomRight = new MechanismLigament2d("Intake Bottom Right", 1, 0);
        intakeBottomLeft = new MechanismLigament2d("Intake Bottom Left", 1, 0);

        intakeTopRight = new MechanismLigament2d("Intake Top", 1, 1);
        intakeTopLeft = new MechanismLigament2d("Intake Top", 0, 0);

        conveyorBottomRight = new MechanismLigament2d("Conveyor Bottom Right", 0, 0);
        conveyorBottomLeft = new MechanismLigament2d("Conveyor Bottom Left", 0, 0);
        
        conveyorTopRight = new MechanismLigament2d("Conveyor Top Right", 0, 0);
        conveyorTopLeft = new MechanismLigament2d("Conveyor Top Left", 0, 0);

        conveyorTopRight = new MechanismLigament2d("Conveyor Top Right", 0, 0);
        conveyorTopLeft = new MechanismLigament2d("Conveyor Top Left", 0, 0);

        shooterBottomRight = new MechanismLigament2d("Shooter Bottom Right", 0, 0);
        shooterBottomLeft = new MechanismLigament2d("Shooter Bottom Left", 0, 0);

        shooterTopRight = new MechanismLigament2d("Shooter Top Right", 0, 0);
        shooterTopLeft = new MechanismLigament2d("Shooter Top Left", 0, 0);

        conveyorIRSensor = new MechanismLigament2d("Conveyor IR Sensor", 1, 0);

        rollerBottomLeft = new MechanismLigament2d("Intake Roller Bottom Left", 0, 0);
        rollerBottomMiddle = new MechanismLigament2d("Intake Roller Bottom Middle", 0, 0);
        rollerBottomRight = new MechanismLigament2d("Intake Roller Bottom Right", 0, 0);

        rollerTopLeft = new MechanismLigament2d("Intake Roller Top Left", 0, 0);
        rollerTopMiddle = new MechanismLigament2d("Intake Roller Top Middle", 0, 0);
        rollerTopRight = new MechanismLigament2d("Intake Roller Top Right", 0, 0);

        // initializing roots
        root_intakeBottomRight = intake.getRoot("Intake Bottom Right Root", 1, 0);
        root_intakeBottomLeft = intake.getRoot("Intake Bottom Left Root", 1, 1);
        
        root_intakeTopRight = intake.getRoot("Intake Top Right Root", 1.5 , 2);
        root_intakeTopLeft = intake.getRoot("Intake Top Left Root", 2, 3);
    
        root_conveyorBottomRight = intake.getRoot("Conveyor Bottom Right Root", 3 ,4.5);
        root_conveyorBottomLeft = intake.getRoot("Conveyor Bottom Left", 4, 5);

        root_conveyorTopRight = intake.getRoot("Conveyor Top Right Root", 4.5, 6.5);
        root_conveyorTopLeft = intake.getRoot("Conveyor Top Left Root", 5, 7);
        
        root_shooterBottomRight = intake.getRoot("Shooter Bottom Right Root", 6, 8);
        root_shooterBottomLeft = intake.getRoot("Shooter Bottom Left Root", 6.5, 8.5);
        
        root_shooterTopRight = intake.getRoot("Shooter Top Right Root", 7 ,9);
        root_shooterTopLeft = intake.getRoot("Shooter Top Left Root", 7.5 , 10);

        root_conveyorIRSensor = intake.getRoot("Conveyor IR Sensor Root", 8, 10.5);

        root_rollerBottomLeft = intake.getRoot("Roller Bottom Left", 0, 0);
        root_rollerBottomMiddle = intake.getRoot("Roller Bottom Middle", 0, 0);
        root_rollerBottomRight = intake.getRoot("Roller Bottom Right", 0, 0);

        root_rollerTopLeft = intake.getRoot("Roller Top Left", 0, 0);
        root_rollerTopMiddle = intake.getRoot("Roller Top Middle", 0, 0);
        root_rollerTopRight = intake.getRoot("Roller Top Right", 0, 0);

        // appending ligaments
        root_intakeBottomRight.append(intakeBottomRight);
        root_intakeBottomLeft.append(intakeBottomLeft);

        root_intakeTopRight.append(intakeTopRight);
        root_intakeTopLeft.append(intakeTopLeft);
        
        root_conveyorBottomRight.append(conveyorBottomRight);
        root_conveyorBottomLeft.append(conveyorBottomLeft);
        
        root_conveyorTopRight.append(conveyorTopRight);
        root_conveyorTopLeft.append(conveyorTopLeft);
    
        root_shooterBottomRight.append(shooterBottomRight);
        root_shooterBottomLeft.append(shooterBottomLeft);

        root_shooterTopRight.append(shooterTopRight);
        root_shooterTopLeft.append(shooterTopLeft);

        root_conveyorIRSensor.append(conveyorIRSensor);

        root_rollerBottomLeft.append(rollerBottomLeft);
        root_rollerBottomMiddle.append(rollerBottomMiddle);
        root_rollerBottomRight.append(rollerBottomRight);

        root_rollerTopLeft.append(rollerTopLeft);
        root_rollerTopMiddle.append(rollerTopMiddle);
        root_rollerTopRight.append(rollerTopRight);
        
        SmartDashboard.putData("Visualizers/Intake Visualizer", intake);
    }

    public void update(boolean conveyorIR, double intakeSpeed, double conveyorSpeed, double shooterSpeed) {
        if (conveyorIR) {
            conveyorIRSensor.setColor(green);
        } else {
            conveyorIRSensor.setColor(red);
        }

        // intake rollers
        rollerBottomLeft.setAngle(intakeAngle);
        rollerTopLeft.setAngle(-intakeAngle);
        intakeAngle += intakeSpeed;
        
        // conveyor rollers
        conveyorAngle += conveyorSpeed;
        rollerBottomMiddle.setAngle(conveyorAngle);
        rollerTopMiddle.setAngle(-conveyorAngle);
        
        // shooter rollers
        shooterAngle += shooterSpeed;
        rollerBottomRight.setAngle(shooterAngle);
        rollerTopRight.setAngle(-shooterAngle);


        
    }
    
}