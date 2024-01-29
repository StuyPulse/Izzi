package com.stuypulse.robot.subsystems.swerve.modules;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
public class SwerveModuleImpl extends SwerveModule {

    private final Rotation2d angleOffset;
    
    private final CANSparkMax turnMotor;
    private final CANSparkMax driveMotor;

    private final RelativeEncoder driveEncoder;
    private final CANcoder turnEncoder;

    private final Controller driveController;
    private final AngleController angleController;

    /**
     * Creates a new Swerve Module
     * @param id id of the module
     * @param offset offset of the module
     * @param driveId id of the drive motor
     * @param angleOffset offset of the angle 
     * @param turnId id of the turn motor
     */
    public SwerveModuleImpl(String id, Translation2d offset, int driveID, Rotation2d angleOffset, int turnID, int encoderID) {
        super(id, offset);
        
        this.angleOffset = angleOffset;
        
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

        Motors.Swerve.DRIVE_CONFIG.configure(driveMotor);
        Motors.Swerve.TURN_CONFIG.configure(turnMotor);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        driveEncoder.setPosition(0);
        
        turnEncoder = new CANcoder(encoderID);

        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
            .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());

        angleController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            .setSetpointFilter(new ARateLimit(Swerve.MAX_TURNING));
    }

    @Override
    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().getValueAsDouble()).minus(angleOffset);
    }

    @Override 
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }    

    @Override
    public void periodic() {
        super.periodic();

        driveController.update(
            getTargetState().speedMetersPerSecond,
            getVelocity()
        );

        angleController.update(
            Angle.fromRotation2d(getTargetState().angle),
            Angle.fromRotation2d(getAngle())
        );

        if (Math.abs(driveController.getOutput()) < Settings.Swerve.MODULE_VELOCITY_DEADBAND.get()) {
            driveMotor.setVoltage(0);
            turnMotor.setVoltage(0);
        } else {
            driveMotor.setVoltage(driveController.getOutput());
            turnMotor.setVoltage(angleController.getOutput());
        }
        
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Drive Voltage", driveController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Turn Voltage", angleController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Angle Error", angleController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Raw Encoder Angle", Units.rotationsToDegrees(turnEncoder.getAbsolutePosition().getValueAsDouble()));
    }
}