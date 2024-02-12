package com.stuypulse.robot.subsystems.leds.instructions;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.LED;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDAlign implements LEDInstruction {

    private final LEDController ledController;
    private final Odometry odometry;
    private final Pose2d startPose;
    
    private final BStream isXAligned;
    private final BStream isYAligned;
    private final BStream isThetaAligned;

    public LEDAlign() {
        startPose = (RobotContainer.getAutonomousCommandNameStatic().equals("DoNothingAuton") ? new Pose2d() : PathPlannerAuto.getStaringPoseFromAutoFile(RobotContainer.getAutonomousCommandNameStatic())); 
        odometry = Odometry.getInstance();
        ledController = LEDController.getInstance();

        isXAligned = BStream.create(this::isXAligned)     
            .filtered(new BDebounceRC.Both(Alignment.DEBOUNCE_TIME));
        isYAligned = BStream.create(this::isYAligned)     
            .filtered(new BDebounceRC.Both(Alignment.DEBOUNCE_TIME));
        isThetaAligned = BStream.create(this::isThetaAligned)     
            .filtered(new BDebounceRC.Both(Alignment.DEBOUNCE_TIME));
    }
   
    public boolean isXAligned() {
        return Math.abs(odometry.getPose().getX() - startPose.getX()) < Settings.Alignment.X_TOLERANCE.get();
    }

    public boolean isYAligned() {
        return Math.abs(odometry.getPose().getY() - startPose.getY()) < Settings.Alignment.Y_TOLERANCE.get();
    }
    
    public boolean isThetaAligned() {
        return Math.abs(odometry.getPose().getRotation().getDegrees() - startPose.getRotation().getDegrees()) < Settings.Alignment.ANGLE_TOLERANCE.get();
    }
 
    @Override
    public void setLED(AddressableLEDBuffer ledsBuffer) {
        Pose2d pose = odometry.getPose();
        int middleLEDindex = ledsBuffer.getLength() / 2;
        ledsBuffer.setLED(middleLEDindex, Color.kBlack);
        
        if (isXAligned() && isYAligned() && isThetaAligned()) {
            ledController.runLEDInstruction(LEDInstructions.RAINBOW);
        }
        else {
            int index = middleLEDindex;

            if (!isXAligned.get()) {
                ledController.runLEDInstruction(LEDInstructions.RED);
                index = linearInterp(pose.getX(), startPose.getX(), LED.TRANSLATION_SPREAD.get());
            } 
            if (!isYAligned.get() && isXAligned()) {
                ledController.runLEDInstruction(LEDInstructions.GREEN);
                index = linearInterp(pose.getY(), startPose.getY(), LED.TRANSLATION_SPREAD.get());
            }
            if (!isThetaAligned.get() && isXAligned() && isYAligned()) {
                ledController.runLEDInstruction(LEDInstructions.DARK_BLUE);
                index = linearInterp(pose.getRotation().getDegrees(), startPose.getRotation().getDegrees(), LED.ROTATION_SPREAD.get());
            }
            
            ledsBuffer.setLED(index, Color.kWhite);
        }
    }

    private int linearInterp(double robotMeasurement, double targetPos, double spread) {
        double lowerBound = targetPos - spread;
        double upperBound = targetPos + spread;
        if (robotMeasurement < lowerBound) {
            return 0;
        }
        if (robotMeasurement > upperBound) {
            return Settings.LED.LED_LENGTH - 1;
        }
        return (int) (Settings.LED.LED_LENGTH * (robotMeasurement - lowerBound) / (spread * 2));
    }
}
