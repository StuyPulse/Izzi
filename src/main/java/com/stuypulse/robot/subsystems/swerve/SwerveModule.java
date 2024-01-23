package com.stuypulse.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*   
* SwerveModule.java
*  Fields:
*   - id: String 
*   - offset: Translation2D
*   - angle offset: Rotation2D 
*   - state: SwerveModuleState
*
*   physical Components:
*   - turn: CANSparkMax
*   - drive: CANSparkMax
*   - driveEncoder: RelativeEncoder
*   - turnAbsoluteEncoder: SparkAbsoluteEncoder
*   
*  Methods:
*   + getState(): SwerveModuleState 
*   + getVelocity(): double
*   + getAngle(): Rotation2D
*   + getID(): String
*   + getModulePosition(): Translation2D
*   + setState(SwerveModuleState state): void
*   + periodic(): void
*
*/
public class SwerveModule extends AbstractSwerveModule {
    private String id;
    
    private Translation2d offset;
    private Rotation2d angleOffset;
    
    private SwerveModuleState state;
    
    private CANSparkMax turnMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private SparkAbsoluteEncoder turnEncoder;

    private Controller driveController;
    private AngleController angleController;

    /**
     * Creates a new Swerve Module
     * @param id id of the module
     * @param offset offset of the module
     * @param driveId id of the drive motor
     * @param angleOffset offset of the angle 
     * @param turnId id of the turn motor
     */
    public SwerveModule(String id, Translation2d offset, int driveId, Rotation2d angleOffset, int turnId){
        this.id = id;
        this.offset = offset;
        this.angleOffset = angleOffset;

        this.state = new SwerveModuleState();
        
        this.driveMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);

        this.driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Settings.Swerve.Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Settings.Swerve.Encoder.Drive.VELOCITY_CONVERSION);
        
        this.turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        turnEncoder.setPositionConversionFactor(Settings.Swerve.Encoder.Turn.POSITION_CONVERSION);
        turnEncoder.setVelocityConversionFactor(Settings.Swerve.Encoder.Turn.VELOCITY_CONVERSION);
        
        this.driveController = new PIDController(Settings.Swerve.Controller.Drive.kP, Settings.Swerve.Controller.Drive.kI, Settings.Swerve.Controller.Drive.kD)
            .add(new MotorFeedforward(Settings.Swerve.Controller.Drive.kS, Settings.Swerve.Controller.Drive.kV, Settings.Swerve.Controller.Drive.kA).velocity());
    
        this.angleController = new AnglePIDController(Settings.Swerve.Controller.Turn.kP, Settings.Swerve.Controller.Turn.kI, Settings.Swerve.Controller.Turn.kD);

        this.driveEncoder.setPosition(0);
        this.turnEncoder.setZeroOffset(this.angleOffset.getRotations());

        Motors.Swerve.DRIVE_CONFIG.configure(driveMotor);
        Motors.Swerve.TURN_CONFIG.configure(turnMotor);
    }
    
    @Override
    public SwerveModuleState getState() {
        return state;
    }

    @Override
    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getPosition());
    }

    @Override
    public String getId() {
        return id;
    }

    @Override
    public Translation2d getModuleOffset() {
        return offset;
    }

    @Override 
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }    
    
    @Override
    public void setState(SwerveModuleState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        // Update Controllers
        driveMotor.setVoltage(driveController.update(state.speedMetersPerSecond, getVelocity()));
        turnMotor.setVoltage(angleController.update(Angle.fromRotation2d(state.angle), Angle.fromRotation2d(getAngle())));

        //Logging
        SmartDashboard.putNumber("Swerve/" + id + "/Raw Angle (deg)", Units.rotationsToDegrees(turnEncoder.getPosition()));
        SmartDashboard.putNumber("Swerve/" + id + "/Target Angle", state.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Error", angleController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Voltage", angleController.getOutput());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Current", turnMotor.getOutputCurrent());
        SmartDashboard.putNumber("Swerve/" + id + "/Target Velocity", state.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Error", driveController.getError());
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Voltage", driveController.getOutput());
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Current", driveMotor.getOutputCurrent());
    }
}