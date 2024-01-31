package com.stuypulse.robot.commands.leds;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.constants.LEDColor;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.instructions.LEDOdometry;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.util.SLColor;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDAlign extends Command {
    private final LEDController ledController;
    private final Odometry odometry;
    private final Pose2d startPose;
    
    private final BStream isXAligned;
    private final BStream isYAligned;
    private final BStream isThetaAligned;

    public LEDAlign(PathPlannerAuto auton) {
        startPose = PathPlannerAuto.getStaringPoseFromAutoFile(auton.getName());
        odometry = Odometry.getInstance();
        ledController = LEDController.getInstance();

        isXAligned = BStream.create(this::isXAligned)     
            .filtered(new BDebounceRC.Both(Alignment.DEBOUNCE_TIME.get()));
        isYAligned = BStream.create(this::isYAligned)     
            .filtered(new BDebounceRC.Both(Alignment.DEBOUNCE_TIME.get()));
        isThetaAligned = BStream.create(this::isThetaAligned)     
            .filtered(new BDebounceRC.Both(Alignment.DEBOUNCE_TIME.get()));

    }
   
    public boolean isXAligned() {
        return odometry.getPose().getX() == startPose.getX();
    }

    public boolean isYAligned() {
        return odometry.getPose().getY() == startPose.getY();
    }
    
    public boolean isThetaAligned() {
        return odometry.getPose().getRotation().getDegrees()  == startPose.getRotation().getDegrees();
    }

    @Override
    public void execute() {
        //TODO: finish it 
        if (!isXAligned.get()) {

        } 
        else if (!isYAligned.get()) {
            
        } 
        else if (!isThetaAligned.get()) {
           
        } 
    }

    @Override
    public boolean isFinished() {
        return isXAligned.get() && isYAligned.get() && isThetaAligned.get();
    }

    @Override
    public void end(boolean interrupted) {
        ledController.forceSetLED(LEDColor.RAINBOW);
    }

    // helper method for LEDOdometry
    public double getTargetMeasurement() {
        if (!isXAligned.get()) {
            return startPose.getX();
        } 
        else if (!isYAligned.get()) {
            return startPose.getY();
        } 
        else if (!isThetaAligned.get()) {
            return startPose.getRotation().getDegrees();
        } 
        else {
            return -1;
        }
    }

}
