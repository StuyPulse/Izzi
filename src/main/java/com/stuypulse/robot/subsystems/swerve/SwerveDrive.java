package com.stuypulse.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.modules.AbstractSwerveModule;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*   
*  Fields: 
*    - swerveModules:  AbstractSwerveModule...
*    - kinematics:  SwerveDriveKinematics
*    - gyro: AHRS
*    - modules2D: FieldObject2D[]
*
*   Tasks:
*    - drive
*    - followDrive 
*    - trackingDrive
*    - aligning (Trap, Speaker, Amp)
*    - GTADrive (shooting while driving)
*
*   Methods:
*    + singleton
*    + initFieldObject(Field2D field): void
*    + getModulePositions(): Translation2D[]
*    + getModuleStates(): SwerveModuleStates[]
*    + getModuleOffsets(): Rotation2D[]
*    + getChassisSpeeds(): ChassisSpeed[]
*    + setModuleStates(SwerveModuleState... states): void
*    + setChassisSpeed(ChassisSpeed): void 
*    + drive(double, Rotation2D)
*    + stop(double, Rotation2D)
*
*  SwerveDrive.java
*   Methods:
*    - getGyroAngle(): Rotation2D
*    - getGyroYaw(): Rotation2D
*    - getGyroPitch(): Rotation2D
*    - getGyroRoll(): Rotation2D
*    - getKinematics(): SwerveDriveKinematics
*    + periodic(): void
*    
*
*/
public class SwerveDrive extends AbstractSwerveDrive {

    private AbstractSwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private AHRS gyro;
    private FieldObject2d[] modules2D;
    

    /**
     * Creates a new Swerve Drive using the provided modules
     * @param modules the modules to use
     */
    public SwerveDrive(AbstractSwerveModule... modules){
        this.modules = modules;
        this.kinematics = new SwerveDriveKinematics(getModuleOffsets());
        this.gyro = new AHRS(SPI.Port.kMXP);
        this.modules2D = new FieldObject2d[modules.length];
    }   

    @Override
    public void initFieldObject(Field2d field) {
        for (int i = 0; i < modules.length; i++){
            modules2D[i] = field.getObject(modules[i].getId() + "-2d");
        }
    }


    /*Getters */
    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
    
    @Override
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i< modules.length; i++){
            states[i] = modules[i].getState();
        }
        return states;
    }
    
    @Override
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] offsets = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++){
            offsets[i] = modules[i].getModulePosition();
        }
        return offsets;
    }

    @Override
    public Translation2d[] getModuleOffsets() {
        Translation2d[] offsets = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            offsets[i] = modules[i].getModuleOffset();
        }
        return offsets;
    }
        
    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }
    
    /*Setters */

    private static SwerveModuleState filterModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) > Swerve.MODULE_VELOCITY_DEADBAND.get())
            return state;

        return new SwerveModuleState(0, state.angle);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("Provided incorrect number of states for swerve drive modules");
        }
        
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_MODULE_SPEED.get());

        for (int i = 0; i < modules.length; i++){
            modules[i].setState(filterModuleState(states[i]));
        }
    }
    
    @Override
    public void setChassisSpeed(ChassisSpeeds robotSpeeds) {
        setModuleStates(kinematics.toSwerveModuleStates(robotSpeeds));
    }

    /*Drive Functions*/
    @Override
    public void drive(Vector2D velocity, double rotation) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.x, -velocity.y,
            rotation,
            Odometry.getInstance().getPose().getRotation()
        );
 
        Pose2d pose = new Pose2d(
            Settings.DT * speeds.vxMetersPerSecond,
            Settings.DT * speeds.vyMetersPerSecond,
            Rotation2d.fromDegrees(Settings.DT * speeds.omegaRadiansPerSecond)
        );

        Twist2d twistVel = new Pose2d().log(pose);   

        setChassisSpeed( new ChassisSpeeds(
            twistVel.dx / Settings.DT,
            twistVel.dy / Settings.DT,
            twistVel.dtheta / Settings.DT
        ));
    }

    @Override
    public void stop(double Rotation2D) {
        setChassisSpeed(new ChassisSpeeds());
    } 

    /*Gyro */
    @Override
    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }
    
    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public double getForwardAccelerationGs() {
        return gyro.getWorldLinearAccelY();
    }
    
    @Override
    public void periodic() {
        Odometry odometry = Odometry.getInstance();
        Pose2d pose = odometry.getPose();
        Rotation2d angle = pose.getRotation();

        for (int i = 0; i < modules.length; i++) {
            modules2D[i].setPose(new Pose2d(
                pose.getTranslation().plus(modules[i].getModuleOffset().rotateBy(angle)),
                modules[i].getAngle().plus(angle)
            ));
        }

        SmartDashboard.putNumber("Swerve/Gyro/Angle (deg)", getGyroAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro/Pitch (deg)", getGyroPitch().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro/Roll (deg)", getGyroRoll().getDegrees());

        SmartDashboard.putNumber("Swerve/Forward Acceleration  (Gs)", getForwardAccelerationGs());
        SmartDashboard.putNumber("Swerve/X Acceleration (Gs)", gyro.getWorldLinearAccelX());
        SmartDashboard.putNumber("Swerve/Y Acceleration (Gs)", gyro.getWorldLinearAccelY());
        SmartDashboard.putNumber("Swerve/Z Acceleration (Gs)", gyro.getWorldLinearAccelZ());
    }

    @Override 
    public void simulationPeriodic() {
        //show gyro angle in simulation
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
        gyro.setAngleAdjustment(speeds.omegaRadiansPerSecond * Settings.DT);
    }
}