/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.stuypulse.robot.commands.*;
import com.stuypulse.robot.commands.leds.*;
import com.stuypulse.robot.commands.amper.*;
import com.stuypulse.robot.commands.auton.*;
import com.stuypulse.robot.commands.climber.*;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.notealignment.SwerveDriveNoteAlignedDrive;
import com.stuypulse.robot.commands.shooter.*;
import com.stuypulse.robot.commands.conveyor.*;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Settings.Driver;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.subsystems.vision.NoteVision;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import com.stuypulse.robot.subsystems.climber.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    public final AprilTagVision vision = AprilTagVision.getInstance();
    public final NoteVision noteVision = NoteVision.getInstance();
    public final Odometry odometry = Odometry.getInstance();
    
    public final Climber climber = Climber.getInstance();
    public final Amper amper = Amper.getInstance();
    public final LEDController leds = LEDController.getInstance();
    public final Conveyor conveyor = Conveyor.getInstance();
    public final Intake intake = Intake.getInstance();
    public final Shooter shooter = Shooter.getInstance();
    public final SwerveDrive swerve = SwerveDrive.getInstance();
  
    // Autons
    private static SendableChooser<Command> autonChooser;

    // RobotContainer
    public RobotContainer() {
        swerve.configureAutoBuilder();

        configureDefaultCommands();
        configureNamedCommands();
        configureButtonBindings();
        configureAutons();

        SmartDashboard.putData("Gamepads/Driver", driver);
        SmartDashboard.putData("Gamepads/Operator", operator);
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
    }

    /**********************/
    /*** NAMED COMMANDS ***/
    /**********************/

    private void configureNamedCommands() {}

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        configureOperatorBindings();
        configureDriverBindings();    
    }

    private void configureDriverBindings() {
        driver.getRightTriggerButton()
            .whileTrue(new IntakeAcquire())
            .whileTrue(new SwerveDriveNoteAlignedDrive(driver));

        // score speaker
        driver.getRightBumper()
            .whileTrue(new ConveyorToShooter()
                .alongWith(new SwerveDrivePathFindTo(Field.getSpeakerPathFindPose()).get())
                .andThen(new SwerveDriveToShoot())
                .andThen(new ConveyorShoot()))
            .onFalse(new ConveyorStop());
        // score amp
        driver.getLeftBumper()
            .whileTrue(new ConveyorToAmp()
                .alongWith(new SwerveDrivePathFindTo(Field.getAmpPathFindPose()).get())
                .andThen(new SwerveDriveAmpAlign())
                .andThen(new AmperScore()))
            .onFalse(new AmperStop());

        // score speaker no align
        driver.getStartButton()
            .whileTrue(new ConveyorToShooter()
                .andThen(new ConveyorShoot()))
            .onFalse(new ConveyorStop());
        // score amp no align
        driver.getSelectButton()
            .whileTrue(new ConveyorToAmp()
                .andThen(new AmperScore()))
            .onFalse(new AmperStop());
    
        driver.getDPadUp()
            .onTrue(new ClimberToTop());
        driver.getDPadDown()
            .onTrue(new ClimberToBottom());
        
        driver.getRightButton()
            .whileTrue(new ClimberSetupRoutine());
        driver.getBottomButton()
            .whileTrue(new ClimberScoreRoutine());
        
        driver.getTopButton()
            .onTrue(new BuzzController(driver, Assist.BUZZ_INTENSITY))
            .onTrue(new SwerveDriveAutomatic(driver)
                .andThen(new BuzzController(driver, Assist.BUZZ_INTENSITY))
                .andThen(new WaitCommand(Driver.Drive.BUZZ_DURATION))
                .andThen(new BuzzController(driver, Assist.BUZZ_INTENSITY)));
    }

    private void configureOperatorBindings() {
        new Trigger(() -> operator.getLeftStick().magnitude() > Settings.Operator.DEADBAND.get())
            .whileTrue(new ClimberDrive(operator));

        new Trigger(() -> operator.getRightStick().magnitude() > Settings.Operator.DEADBAND.get())
            .whileTrue(new AmperLiftDrive(operator));
        
        operator.getLeftTriggerButton()
            .whileTrue(new ConveyorOuttake());
        operator.getRightTriggerButton()
            .whileTrue(new IntakeAcquire());

        operator.getLeftBumper()
            .onTrue(ConveyorToAmp.withCheckLift());
        operator.getRightBumper()
            .onTrue(new ConveyorToShooter());

        operator.getTopButton()
            .onTrue(new AmperScore())
            .onFalse(new AmperStop())
            .onTrue(new ConveyorToShooter())
            .onFalse(new ConveyorStop());

        operator.getDPadUp()
            .whileTrue(new AmperLiftFineAdjust(operator));
        operator.getDPadDown()
            .whileTrue(new AmperLiftFineAdjust(operator));

        operator.getDPadRight()
            .onTrue(new ClimberToTop());
        operator.getDPadLeft()
            .onTrue(new ClimberToBottom());

        operator.getLeftButton()
            .onTrue(new AmperToHeight(Settings.Amper.Lift.TRAP_SCORE_HEIGHT.get()));
        operator.getRightButton()
            .onTrue(new AmperToHeight(Settings.Amper.Lift.AMP_SCORE_HEIGHT.get()));
        operator.getBottomButton()
            .onTrue(new AmperToHeight(Settings.Amper.Lift.MIN_HEIGHT));
        
        // human player attention button
        operator.getSelectButton()
            .whileTrue(new LEDSet(LEDInstructions.PULSE_PURPLE))
            .onFalse(new LEDSet(LEDInstructions.RAINBOW));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser = AutoBuilder.buildAutoChooser();

        //need auton for testing leds
        autonChooser.setDefaultOption("DoNothingAuton", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public static String getAutonomousCommandNameStatic() {
        return autonChooser.getSelected().getName();
    }
}
