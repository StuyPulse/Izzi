package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Motors.Swerve;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.Command;
/*
 * when a button is pressed this command is activated
 * point at either speaker or note (can add in pointing at amp later)
 * - use SwerveDriveToShoot command to point at speaker 
 * - pointing logic
 * -first check if there is a note in the speaker, intake, or amp 
 * --if so, point at the speaker 
 * -second check if there is a note (hasNoteData from LLNoteVision.java)
 * --if so, point at the note
 * --idt theres a function to point at the note right now but vision will probably add it in so we can j add aplaceholder or smth
 * -if there isnt a note nearby then just point at the speaker 
 * 
 * inside isFinisehd commanbd (stops if this is true)
 * - check if the joystick is moving (out of a certain range), or if a button is pressed
 * 
 * later: 
 * - try it out in a sim 
 * - in robotcontainer: bind a button to start the SwerveDriveAutomatic 
 * 
 */
public class SwerveDriveAutomatic extends Command {
    private SwerveDrive swerve;
    private final Gamepad driver;
    
    public SwerveDriveAutomatic(Gamepad driver) {
        this.driver = driver;
        swerve = SwerveDrive.getInstance();
        addRequirements(swerve);

    }


    @Override
    public void execute() {
        //logic here 

    }

    @Override
    public boolean isFinished() {
        return false; //change later 
    }

}