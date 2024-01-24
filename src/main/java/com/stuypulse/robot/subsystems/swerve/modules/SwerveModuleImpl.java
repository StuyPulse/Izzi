package com.stuypulse.robot.subsystems.swerve.modules;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;

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

    /**
     * Creates a new Swerve Module
     * @param id id of the module
     * @param offset offset of the module
     * @param driveId id of the drive motor
     * @param angleOffset offset of the angle 
     * @param turnId id of the turn motor
     */
    public SwerveModuleImpl(String id, Translation2d offset, int driveID, Rotation2d angleOffset, int turnID){
        super(id, offset);
        
        this.angleOffset = angleOffset;        
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        driveEncoder.setPosition(0);
        
        turnEncoder = new CANcoder(turnID);

        Motors.Swerve.DRIVE_CONFIG.configure(driveMotor);
        Motors.Swerve.TURN_CONFIG.configure(turnMotor);
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
	protected void setVoltageImpl(double driveVoltage, double turnVoltage) {
		driveMotor.setVoltage(driveVoltage);
        turnMotor.setVoltage(turnVoltage);
	}

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Swerve/Modules/" + this.getId() + "/Raw Encoder Angle", Units.rotationsToDegrees(turnEncoder.getAbsolutePosition().getValueAsDouble()));
    }
}