package com.stuypulse.robot.subsystems.amp;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static com.stuypulse.robot.constants.Ports.Amp.*;
import edu.wpi.first.wpilibj.DigitalInput;


public class AmperImpl extends Amper {
    private final CANSparkMax scoreMotor;
    private final CANSparkMax liftMotor;
    private final RelativeEncoder liftEncoder;

    private final DigitalInput alignedSwitch;
    private final DigitalInput minSwitch;
    private final DigitalInput maxSwitch; 

    public AmperImpl() {
        // this calls the constructor from the abstract class
        super();

        scoreMotor = new CANSparkMax(SCORE, MotorType.kBrushless);
        liftMotor = new CANSparkMax(LIFT, MotorType.kBrushless);
        liftEncoder = liftMotor.getEncoder();

        alignedSwitch = new DigitalInput(ALIGNED_SWITCH_CHANNEL);
        minSwitch = new DigitalInput(MIN_LIFT_CHANNEL);
        maxSwitch = new DigitalInput(MAX_LIFT_CHANNEL);
    }

    @Override
    public boolean hasNote() {
        return false;
    }

    @Override
    public void acquire() {
    }

    @Override
    public void deacquire() {
    }

    @Override
    public void setTargetHeight(double height) {
        targetHeight.set(height);
    }

    @Override
    public void stopLift() {
        liftMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        // liftController.update returns the voltage output after updating the controller :)
        liftMotor.setVoltage(liftController.update(targetHeight.get(), liftEncoder.getPosition()));
    }
}
