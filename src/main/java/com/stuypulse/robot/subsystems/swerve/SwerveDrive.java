package com.stuypulse.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.BackLeft;
import com.stuypulse.robot.constants.Settings.Swerve.BackRight;
import com.stuypulse.robot.constants.Settings.Swerve.FrontLeft;
import com.stuypulse.robot.constants.Settings.Swerve.FrontRight;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.modules.SwerveModuleSim;
import com.stuypulse.robot.subsystems.swerve.modules.SwerveModule;
import com.stuypulse.robot.subsystems.swerve.modules.SwerveModuleImpl;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
public class SwerveDrive extends SubsystemBase {

    private static final SwerveDrive instance;

    static {
        if (RobotBase.isSimulation()) {
            instance = new SwerveDrive(
                new SwerveModuleSim(FrontRight.ID, FrontRight.MODULE_OFFSET),
                new SwerveModuleSim(FrontLeft.ID, FrontLeft.MODULE_OFFSET),
                new SwerveModuleSim(BackLeft.ID, BackLeft.MODULE_OFFSET),
                new SwerveModuleSim(BackRight.ID, BackRight.MODULE_OFFSET)
            );
        }     
        else {
            instance = new SwerveDrive(  
                new SwerveModuleImpl(FrontRight.ID, FrontRight.MODULE_OFFSET, Ports.Swerve.FrontRight.TURN, FrontRight.ABSOLUTE_OFFSET, Ports.Swerve.FrontRight.DRIVE, Ports.Swerve.FrontRight.ENCODER),
                new SwerveModuleImpl(FrontLeft.ID, FrontLeft.MODULE_OFFSET, Ports.Swerve.FrontLeft.TURN, FrontLeft.ABSOLUTE_OFFSET, Ports.Swerve.FrontLeft.DRIVE, Ports.Swerve.FrontLeft.ENCODER),
                new SwerveModuleImpl(BackLeft.ID, BackLeft.MODULE_OFFSET, Ports.Swerve.BackLeft.TURN, BackLeft.ABSOLUTE_OFFSET, Ports.Swerve.BackLeft.DRIVE, Ports.Swerve.FrontLeft.ENCODER),
                new SwerveModuleImpl(BackRight.ID, BackRight.MODULE_OFFSET, Ports.Swerve.BackRight.TURN, BackRight.ABSOLUTE_OFFSET, Ports.Swerve.BackRight.DRIVE, Ports.Swerve.FrontLeft.ENCODER)
            );
        }
    }

    public static SwerveDrive getInstance() {
        return instance;
    }

    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final AHRS gyro;
    private final FieldObject2d[] modules2D;
    

    /**
     * Creates a new Swerve Drive using the provided modules
     * @param modules the modules to use
     */
    public SwerveDrive(SwerveModule... modules) {
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(getModuleOffsets());
        gyro = new AHRS(SPI.Port.kMXP);
        modules2D = new FieldObject2d[modules.length];
    }  
    
    public void configureAutoBuilder() {
        Odometry odometry = Odometry.getInstance();

        AutoBuilder.configureHolonomic(
            odometry::getPose,
            odometry::reset,
            this::getChassisSpeeds, 
            this::setChassisSpeeds, 
            new HolonomicPathFollowerConfig(
                Swerve.Motion.XY, 
                Swerve.Motion.THETA, 
                Swerve.MAX_MODULE_SPEED.get(), 
                Swerve.WIDTH, 
                new ReplanningConfig(true, true)), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            instance
        );

        PathPlannerLogging.setLogActivePathCallback((poses) -> odometry.getField().getObject("path").setPoses(poses));
    }

    public void initFieldObject(Field2d field) {
        for (int i = 0; i < modules.length; i++){
            modules2D[i] = field.getObject(modules[i].getId() + "-2d");
        }
    }

    /** Getters **/
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
    
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i< modules.length; i++){
            states[i] = modules[i].getState();
        }
        return states;
    }
    
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] offsets = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++){
            offsets[i] = modules[i].getModulePosition();
        }
        return offsets;
    }

    public Translation2d[] getModuleOffsets() {
        Translation2d[] offsets = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            offsets[i] = modules[i].getModuleOffset();
        }
        return offsets;
    }
        
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }
    
    /** Setters **/
    public void setModuleStates(SwerveModuleState[] states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("Provided incorrect number of states for swerve drive modules");
        }
        
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_MODULE_SPEED.get());

        for (int i = 0; i < modules.length; i++){
            modules[i].setTargetState(states[i]);
        }
    }
    
    public void setChassisSpeeds(ChassisSpeeds robotSpeeds) {
        setModuleStates(kinematics.toSwerveModuleStates(robotSpeeds));
    }

    public void setXMode() {
        setModuleStates(
            new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(135))
            }
        );
    }

    /** Drive Functions **/
    public void drive(Vector2D velocity, double rotation) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity.y, -velocity.x,
                -rotation,
                Odometry.getInstance().getPose().getRotation());

        Pose2d robotVel = new Pose2d(
            Settings.DT * speeds.vxMetersPerSecond,
            Settings.DT * speeds.vyMetersPerSecond,
            Rotation2d.fromRadians(Settings.DT * speeds.omegaRadiansPerSecond));
        Twist2d twistVel = new Pose2d().log(robotVel);

        setChassisSpeeds(new ChassisSpeeds(
            twistVel.dx / Settings.DT,
            twistVel.dy / Settings.DT,
            twistVel.dtheta / Settings.DT
        ));
    }

    public void stop() {
        setChassisSpeeds(new ChassisSpeeds());
    } 

    /** Gyro **/
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

        SmartDashboard.putNumber("Swerve/Chassis X Speed", getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis Y Speed", getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis Rotation", getChassisSpeeds().omegaRadiansPerSecond);
    }

    @Override
    public void simulationPeriodic() {
        // show gyro angle in simulation
        gyro.setAngleAdjustment(gyro.getAngle() - Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond * Settings.DT));
    }
}