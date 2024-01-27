package com.stuypulse.robot.commands.swerve;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.constants.Settings.Driver.Turn.GyroFeedback;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.RateLimit;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDrive extends Command {
    
    private SwerveDrive swerve;

    private VStream speed; 
    private IStream turn;

    private final Gamepad driver;

    private Optional<Rotation2d> holdAngle;

    public SwerveDriveDrive(Gamepad driver) {
        this.driver = driver;

        swerve = SwerveDrive.getInstance();

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> Settings.vpow(x, Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL.get()),
                new VLowPassFilter(Drive.RC.get())
        );

        turn = IStream.create(driver::getRightX)
            .filtered(
          //      x -> SLMath.deadband(x, Turn.DEADBAND.get()),
          //      x -> SLMath.spow(x, Turn.POWER.get()),
                x -> x * Turn.MAX_TELEOP_TURNING.get()
          //      new RateLimit(Turn.RC.get())
        ); 

        holdAngle = Optional.empty();
        
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        holdAngle = Optional.empty();
    }

    private boolean isWithinTurnDeadband() {
        return Math.abs(turn.get()) < Turn.DEADBAND.get();
    }

    @Override
    public void execute() {
        double angularVel = turn.get();

        if(isWithinTurnDeadband() && holdAngle.isEmpty()){
            holdAngle = Optional.of(swerve.getGyroAngle());
        }
        else {
            holdAngle = Optional.empty(); 
        }

        swerve.drive(speed.get(), angularVel);
    }
}
