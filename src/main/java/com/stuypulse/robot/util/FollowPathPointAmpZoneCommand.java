package com.stuypulse.robot.util;

import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.util.AngleVelocity;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Base command for following a path */
public class FollowPathPointAmpZoneCommand extends Command {
  private final Timer timer = new Timer();
  private final PathPlannerPath originalPath;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final Consumer<ChassisSpeeds> output;
  private final PathFollowingController controller;
  private final ReplanningConfig replanningConfig;
  private final BooleanSupplier shouldFlipPath;

  // For event markers
  private final Map<Command, Boolean> currentEventCommands = new HashMap<>();
  private final List<Pair<Double, Command>> untriggeredEvents = new ArrayList<>();

  private PathPlannerPath path;
  private PathPlannerTrajectory generatedTrajectory;

  // Heading Controller

  private final AngleController headingController;
  private final IStream angleVelocity;

  /**
   * Construct a base path following command
   *
   * @param path The path to follow
   * @param poseSupplier Function that supplies the current field-relative pose of the robot
   * @param speedsSupplier Function that supplies the current robot-relative chassis speeds
   * @param outputRobotRelative Function that will apply the robot-relative output speeds of this
   *     command
   * @param controller Path following controller that will be used to follow the path
   * @param replanningConfig Path replanning configuration
   * @param shouldFlipPath Should the path be flipped to the other side of the field? This will
   *     maintain a global blue alliance origin.
   * @param requirements Subsystems required by this command, usually just the drive subsystem
   */
  public FollowPathPointAmpZoneCommand(
      PathPlannerPath path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      Consumer<ChassisSpeeds> outputRobotRelative,
      PathFollowingController controller,
      ReplanningConfig replanningConfig,
      BooleanSupplier shouldFlipPath,
      Subsystem... requirements) {
    this.originalPath = path;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.output = outputRobotRelative;
    this.controller = controller;
    this.replanningConfig = replanningConfig;
    this.shouldFlipPath = shouldFlipPath;


    headingController = new AnglePIDController(Assist.kP, Assist.kI, Assist.kD)
      .setOutputFilter(x -> -x);

    AngleVelocity derivative = new AngleVelocity();

    angleVelocity = IStream.create(() -> derivative.get(Angle.fromRotation2d(getTargetAngle())))
        .filtered(new LowPassFilter(Assist.ANGLE_DERIV_RC))
        // make angleVelocity contribute less once distance is less than REDUCED_FF_DIST
        // so that angular velocity doesn't oscillate
        .filtered(x -> x * Math.min(1, getDistanceToTarget() / Assist.REDUCED_FF_DIST))
        .filtered(x -> -x);

    Set<Subsystem> driveRequirements = Set.of(requirements);
    m_requirements.addAll(driveRequirements);

    for (EventMarker marker : this.originalPath.getEventMarkers()) {
      var reqs = marker.getCommand().getRequirements();

      if (!Collections.disjoint(driveRequirements, reqs)) {
        throw new IllegalArgumentException(
            "Events that are triggered during path following cannot require the drive subsystem");
      }

      m_requirements.addAll(reqs);
    }
  }

    public Rotation2d getTargetAngle() {
        Translation2d robotPose = poseSupplier.get().getTranslation();
        Translation2d ampPose = Field.getAllianceAmpPose().getTranslation();
        return ampPose.minus(robotPose).getAngle()
            .plus(Rotation2d.fromDegrees(180));
    }

    public double getDistanceToTarget() {
        Translation2d robotPose = poseSupplier.get().getTranslation();
        Translation2d ampPose = Field.getAllianceSpeakerPose().getTranslation();
        return ampPose.getDistance(robotPose);
    }

  @Override
  public void initialize() {
    if (shouldFlipPath.getAsBoolean() && !originalPath.preventFlipping) {
      path = originalPath.flipPath();
    } else {
      path = originalPath;
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisSpeeds currentSpeeds = speedsSupplier.get();

    controller.reset(currentPose, currentSpeeds);

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());
    Rotation2d currentHeading =
        new Rotation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    Rotation2d targetHeading =
        path.getPoint(1).position.minus(path.getPoint(0).position).getAngle();
    Rotation2d headingError = currentHeading.minus(targetHeading);
    boolean onHeading =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond) < 0.25
            || Math.abs(headingError.getDegrees()) < 30;

    if (!path.isChoreoPath()
        && replanningConfig.enableInitialReplanning
        && (currentPose.getTranslation().getDistance(path.getPoint(0).position) > 0.25
            || !onHeading)) {
      replanPath(currentPose, currentSpeeds);
    } else {
      generatedTrajectory = path.getTrajectory(currentSpeeds, currentPose.getRotation());
      PathPlannerLogging.logActivePath(path);
      PPLibTelemetry.setCurrentPath(path);
    }

    // Initialize marker stuff
    currentEventCommands.clear();
    untriggeredEvents.clear();
    untriggeredEvents.addAll(generatedTrajectory.getEventCommands());

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double currentTime = timer.get();
    PathPlannerTrajectory.State targetState = generatedTrajectory.sample(currentTime);
    if (!controller.isHolonomic() && path.isReversed()) {
      targetState = targetState.reverse();
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisSpeeds currentSpeeds = speedsSupplier.get();

    if (!path.isChoreoPath() && replanningConfig.enableDynamicReplanning) {
      double previousError = Math.abs(controller.getPositionalError());
      double currentError = currentPose.getTranslation().getDistance(targetState.positionMeters);

      if (currentError >= replanningConfig.dynamicReplanningTotalErrorThreshold
          || currentError - previousError
              >= replanningConfig.dynamicReplanningErrorSpikeThreshold) {
        replanPath(currentPose, currentSpeeds);
        timer.reset();
        targetState = generatedTrajectory.sample(0);
      }
    }

    ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);

    targetSpeeds.omegaRadiansPerSecond += angleVelocity.get()
        + headingController.update(
            Angle.fromRotation2d(getTargetAngle()),
            Angle.fromRotation2d(currentPose.getRotation()));

    double currentVel =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    PPLibTelemetry.setCurrentPose(currentPose);
    PathPlannerLogging.logCurrentPose(currentPose);

    if (controller.isHolonomic()) {
      PPLibTelemetry.setTargetPose(targetState.getTargetHolonomicPose());
      PathPlannerLogging.logTargetPose(targetState.getTargetHolonomicPose());
    } else {
      PPLibTelemetry.setTargetPose(targetState.getDifferentialPose());
      PathPlannerLogging.logTargetPose(targetState.getDifferentialPose());
    }

    PPLibTelemetry.setVelocities(
        currentVel,
        targetState.velocityMps,
        currentSpeeds.omegaRadiansPerSecond,
        targetSpeeds.omegaRadiansPerSecond);
    PPLibTelemetry.setPathInaccuracy(controller.getPositionalError());

    output.accept(targetSpeeds);

    if (!untriggeredEvents.isEmpty() && timer.hasElapsed(untriggeredEvents.get(0).getFirst())) {
      // Time to trigger this event command
      Pair<Double, Command> event = untriggeredEvents.remove(0);

      for (var runningCommand : currentEventCommands.entrySet()) {
        if (!runningCommand.getValue()) {
          continue;
        }

        if (!Collections.disjoint(
            runningCommand.getKey().getRequirements(), event.getSecond().getRequirements())) {
          runningCommand.getKey().end(true);
          runningCommand.setValue(false);
        }
      }

      event.getSecond().initialize();
      currentEventCommands.put(event.getSecond(), true);
    }

    // Run event marker commands
    for (Map.Entry<Command, Boolean> runningCommand : currentEventCommands.entrySet()) {
      if (!runningCommand.getValue()) {
        continue;
      }

      runningCommand.getKey().execute();

      if (runningCommand.getKey().isFinished()) {
        runningCommand.getKey().end(false);
        runningCommand.setValue(false);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(generatedTrajectory.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();

    // Only output 0 speeds when ending a path that is supposed to stop, this allows interrupting
    // the command to smoothly transition into some auto-alignment routine
    if (!interrupted && path.getGoalEndState().getVelocity() < 0.1) {
      output.accept(new ChassisSpeeds());
    }

    PathPlannerLogging.logActivePath(null);

    // End markers
    for (Map.Entry<Command, Boolean> runningCommand : currentEventCommands.entrySet()) {
      if (runningCommand.getValue()) {
        runningCommand.getKey().end(true);
      }
    }
  }

  private void replanPath(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    PathPlannerPath replanned = path.replan(currentPose, currentSpeeds);
    generatedTrajectory = replanned.getTrajectory(currentSpeeds, currentPose.getRotation());
    PathPlannerLogging.logActivePath(replanned);
    PPLibTelemetry.setCurrentPath(replanned);
  }
}
