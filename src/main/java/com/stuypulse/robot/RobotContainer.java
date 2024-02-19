/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import com.stuypulse.robot.commands.*;
import com.stuypulse.robot.commands.amper.*;
import com.stuypulse.robot.commands.auton.*;
import com.stuypulse.robot.commands.climber.*;
import com.stuypulse.robot.commands.conveyor.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.leds.*;
import com.stuypulse.robot.commands.notealignment.SwerveDriveNoteAlignedDrive;
import com.stuypulse.robot.commands.notealignment.SwerveDriveTranslateToNote;
import com.stuypulse.robot.commands.shooter.*;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver;
import com.stuypulse.robot.constants.Settings.Amper.Lift;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.robot.subsystems.amper.Amper;
import com.stuypulse.robot.subsystems.climber.*;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.subsystems.vision.NoteVision;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
        leds.setDefaultCommand(new LEDDefaultMode());
    }

    /**********************/
    /*** NAMED COMMANDS ***/
    /**********************/

    private void configureNamedCommands() {
        NamedCommands.registerCommand("IntakeAcquire", new IntakeAcquireForever());
        NamedCommands.registerCommand("IntakeStop", new IntakeStop());
        NamedCommands.registerCommand(
            "DriveToNote",
            new SwerveDriveDriveToNote()
                .alongWith(new IntakeAcquire())
                .andThen(new IntakeStop()));
        NamedCommands.registerCommand("DriveToShoot", new SwerveDriveToShoot());
        NamedCommands.registerCommand("SetPodiumRangeShot", new ShooterPodiumShot());
        NamedCommands.registerCommand("ConveyorShoot", new ConveyorShootRoutine());
        NamedCommands.registerCommand("TranslateToNote", new SwerveDriveTranslateToNote());
        // NOTE: this command will not change the pose if the alliance changes after deploy (I think)
        NamedCommands.registerCommand("PathFindToShoot", new SwerveDrivePathFindTo(Field.TOP_SHOOT_POSE.get()).get());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        configureOperatorBindings();
        configureDriverBindings();
    }

    private void configureDriverBindings() {
        // intaking with swerve pointing at note
        driver.getRightTriggerButton()
            .whileTrue(new IntakeAcquire())
            .whileTrue(new SwerveDriveNoteAlignedDrive(driver))
            .whileTrue(new LEDSet(LEDInstructions.DARK_BLUE));

        // note to shooter and align
        // then shoot
        driver.getRightBumper()
            .whileTrue(new ConveyorToShooter()
                    .alongWith(new SwerveDriveToShoot()
                        .deadlineWith(new LEDSet(LEDInstructions.GREEN)))
                    // .alongWith(new SwerveDrivePathFindTo(Field.getSpeakerPathFindPose()).get())
                    // .andThen(new SwerveDriveToShoot()
                    //    .deadlineWith(new LEDSet(LEDInstructions.GREEN)))
                    .andThen(new ConveyorShoot()))
            .onFalse(new ConveyorStop());

        // note to amper and align
        // then score
        driver.getLeftBumper()
            .whileTrue(new ConveyorToAmp()
                .alongWith(new SwerveDriveAmpAlign()
                    .deadlineWith(new LEDSet(LEDInstructions.GREEN)))
                .andThen(new AmperScore()))
            .onFalse(new AmperStop());

        // score speaker no align
        // right button
        driver.getStartButton()
            .whileTrue(new ConveyorToShooter()
            .andThen(new ConveyorShoot()))
            .onFalse(new ConveyorStop());
            
        // score amp no align
        // left button
        driver.getSelectButton()
            .whileTrue(new ConveyorToAmp()
                .andThen(new AmperScore()))
            .onFalse(new AmperStop());

        driver.getDPadUp()
            .onTrue(new ClimberToTop());
        driver.getDPadDown()
            .onTrue(new ClimberToBottom());

        driver.getDPadRight()
            .onTrue(new SwerveDriveResetHeading());

        driver.getRightButton()
            .whileTrue(new ClimberSetupRoutine());
        driver.getBottomButton()
            .whileTrue(new ClimberScoreRoutine());
        
        driver.getLeftTriggerButton()
            .onTrue(new IntakeDeacquire())
            .onFalse(new IntakeStop());

        driver.getTopButton()
            // on command start
            .onTrue(new BuzzController(driver, Assist.BUZZ_INTENSITY)
                .deadlineWith(new LEDSet(LEDInstructions.GREEN)))
                
            .onTrue(new SwerveDriveAutomatic(driver)
                // after command end
                .andThen(new BuzzController(driver, Assist.BUZZ_INTENSITY)
                    .deadlineWith(new LEDSet(LEDInstructions.GREEN)))

                .andThen(new WaitCommand(Driver.Drive.BUZZ_DURATION))
                
                .andThen(new BuzzController(driver, Assist.BUZZ_INTENSITY)
                    .deadlineWith(new LEDSet(LEDInstructions.GREEN))));
    }

    private void configureOperatorBindings() {
        new Trigger(() -> operator.getLeftStick().magnitude() > Settings.Operator.DEADBAND.get())
                .whileTrue(new ClimberDrive(operator));

        new Trigger(() -> operator.getRightStick().magnitude() > Settings.Operator.DEADBAND.get())
                .whileTrue(new AmperLiftDrive(operator));

        operator.getLeftTriggerButton()
            .whileTrue(new ConveyorOuttake());
        operator.getRightTriggerButton()
                .whileTrue(new IntakeAcquire())
                .whileTrue(new LEDSet(LEDInstructions.DARK_BLUE));

        operator.getLeftBumper()
            .onTrue(ConveyorToAmp.withCheckLift())
            .onFalse(new ConveyorStop())
            .onFalse(new IntakeStop())
            .onFalse(new AmperStop());
        operator.getRightBumper()
            .onTrue(new ConveyorToShooter())
            .onFalse(new ConveyorStop())
            .onFalse(new IntakeStop());

        operator.getTopButton()
                .onTrue(new AmperScore())
                .onTrue(new ConveyorShoot())
                .onFalse(new AmperStop())
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
            .whileTrue(new LEDSet(LEDInstructions.PULSE_PURPLE));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser = AutoBuilder.buildAutoChooser();

        // need auton for testing leds
        autonChooser.setDefaultOption("DoNothingAuton", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public static String getAutonomousCommandNameStatic() {
        if (autonChooser.getSelected() == null) {
            return "DoNothingAuton";
        }
        
        return autonChooser.getSelected().getName();
    }
}
