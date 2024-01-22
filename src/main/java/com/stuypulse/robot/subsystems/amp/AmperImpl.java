package com.stuypulse.robot.subsystems.amp;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings.Amp.Lift;
import com.stuypulse.robot.constants.Settings.Amp.Score;
import com.stuypulse.stuylib.control.feedback.PIDController;


public class AmperImpl extends Amper {
    private final CANSparkMax scoreMotor;
    private final CANSparkMax liftMotor;
    private final RelativeEncoder liftEncoder;

    private final SparkLimitSwitch alignedSwitch;
    private final SparkLimitSwitch minSwitch;
    private final SparkLimitSwitch maxSwitch; 

    private final PIDController scoreController;
    private final PIDController liftController;
    
    private double currentHeight;
    private double targetHeight;

    public AmperImpl(int port) {
        scoreMotor = new CANSparkMax(port, MotorType.kBrushless);
        liftMotor = new CANSparkMax(port, MotorType.kBrushless);
        liftEncoder = liftMotor.getEncoder();

        alignedSwitch = scoreMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        minSwitch = liftMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        maxSwitch = liftMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        
        scoreController = new PIDController(Score.kP, Score.kI, Score.kD);
        liftController = new PIDController(Lift.kP, Lift.kI, Lift.kD);

        currentHeight = 0;
    }

    @Override
    public boolean hasNote() {
        return false;
    }

    @Override
    public void acquire() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'acquire'");
    }

    @Override
    public void deacquire() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'deacquire'");
    }

    @Override
    public void lift(double height) {
        this.targetHeight = height;
    }

    @Override
    public void stopLift() {
        liftMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        liftController.update(targetHeight, currentHeight);
        liftMotor.setVoltage(liftController.getOutput());
    }
}
