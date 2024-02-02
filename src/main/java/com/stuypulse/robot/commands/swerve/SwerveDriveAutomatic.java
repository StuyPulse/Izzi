package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.*;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.subsystems.conveyor.Conveyor;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.NoteVision;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    
    private final SwerveDrive swerve;
    private final VStream drive;
    private final AngleController controller;
    private final Conveyor conveyor;
    private final NoteVision llNoteVision;
    private final Gamepad driver;
    
    public SwerveDriveAutomatic(Gamepad driver) {
        this.driver = driver;
        swerve = SwerveDrive.getInstance();
        drive = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> Settings.vpow(x, Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL.get()),
                new VLowPassFilter(Drive.RC.get())
        );
        controller = new AnglePIDController(Assist.kP,Assist.kI,Assist.kD);
        conveyor = Conveyor.getInstance();
        llNoteVision = NoteVision.getInstance();

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        //logic here 
        Translation2d currentPose = Odometry.getInstance().getPose().getTranslation();
        Rotation2d currentAngle = currentPose.getAngle();
              
        //if note in speaker 
        if(conveyor.isNoteAtShooter()) {
            Translation2d speakerPose = Field.getAllianceSpeakerPose().getTranslation();
            Translation2d difference = speakerPose.minus(currentPose);
            Rotation2d targetAngle = difference.getAngle();  
                  
            controller.update(Angle.fromDegrees(targetAngle.getDegrees()), Angle.fromDegrees(currentAngle.getDegrees()));
            swerve.drive(drive.get(), controller.getOutput());
        }
        else if (llNoteVision.hasNoteData()) {
            Translation2d notePose = llNoteVision.getEstimatedNotePose();
            Translation2d difference = notePose.minus(currentPose);
            Rotation2d targetAngle = difference.getAngle();
            
            
            controller.update(Angle.fromDegrees(targetAngle.getDegrees()), Angle.fromDegrees(currentAngle.getDegrees()));
            swerve.drive(drive.get(), controller.getOutput());
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(driver.getLeftX()) > Assist.deadband.getAsDouble() || driver.getRawStartButton();
    }

}