// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.VecBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(21.73);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(21.73);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Pose2d pose = new Pose2d(9.45, 4.11, new Rotation2d());
  private Rotation2d lastGyroRotation = new Rotation2d();

  private SwerveDrivePoseEstimator poseEstimator;

  // Field oriented direction in degrees
  private PIDController fieldOrientedDirectionController = new PIDController(0.05, 0.0, 0.0);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    fieldOrientedDirectionController.enableContinuousInput(0, 360.0);


    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      // e.printStackTrace();
      config = new RobotConfig(60, 6.883, new ModuleConfig(0.048, MAX_LINEAR_SPEED, 1.200, new DCMotor(0, 0, 0, 0, 0, 0), 60, 4), TRACK_WIDTH_X);
    }

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        (speeds, feedforwards) -> runVelocity(speeds),
        new PPHolonomicDriveController(
          new PIDConstants(5.0, 0, 0, 0),
          new PIDConstants(5.0, 0, 0, 0)
        ),
        config,
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    // PathPlannerLogging.setLogActivePathCallback(
    //     (activePath) -> {
    //       Logger.recordOutput(
    //           "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
    //     });
    // PathPlannerLogging.setLogTargetPoseCallback(
    //     (targetPose) -> {
    //       Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    //     });

    poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRotation(), getWheelPositions(), pose);
    
  }

  public void periodic() {
    // odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    // odometryLock.unlock();
    // Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }

      setTargetDirection(getRotation().getDegrees());
    }
    // Log empty setpoint states when disabled
    // if (DriverStation.isDisabled()) {
    // Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
    // Logger.recordOutput("SwerveStates/SetpointsOptimized", new
    // SwerveModuleState[] {});
    // }

    // Update odometry
    int deltaCount = Integer.MAX_VALUE;
    // gyroInputs.connected ? gyroInputs.odometryYawPositions.length :
    // Integer.MAX_VALUE;
    for (int i = 0; i < 4; i++) {
      deltaCount = Math.min(deltaCount, modules[i].getPositionDeltas().length);
    }
    for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {
      // Read wheel deltas from each module
      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        wheelDeltas[moduleIndex] = modules[moduleIndex].getPositionDeltas()[deltaIndex];
      }

      // The twist represents the motion of the robot since the last
      // sample in x, y, and theta based only on the modules, without
      // the gyro. The gyro is always disconnected in simulation.
      var twist = kinematics.toTwist2d(wheelDeltas);
      if (gyroInputs.connected) {
        // If the gyro is connected, replace the theta component of the twist
        // with the change in angle since the last sample.
        Rotation2d gyroRotation = gyroInputs.yawPosition;
        twist =
            new Twist2d(
                twist.dx, twist.dy, gyroRotation.minus(lastGyroRotation).getRadians() / deltaCount);
      }
      // Apply the twist (change since last sample) to the current pose
      pose = pose.exp(twist);
    }
    lastGyroRotation = gyroInputs.yawPosition;
    
    updateRobotPosition();
  }

  public SwerveModulePosition[] getWheelPositions() {
    SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      wheelPositions[moduleIndex] = modules[moduleIndex].getPosition();
    }

    return wheelPositions;
  }

  public void resetRotation(double resetDirection) {
    gyroInputs.yawOffset = gyroInputs.realYawPosition.minus(Rotation2d.fromDegrees(resetDirection));
    var currentPose = getPose();
    setTargetDirection(resetDirection);
    setPose(new Pose2d(currentPose.getTranslation(), Rotation2d.fromDegrees(resetDirection)));
    poseEstimator.update(Rotation2d.fromDegrees(resetDirection), getWheelPositions());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Logger.recordOutput("Drive/Speeds", speeds);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    // Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    // Logger.recordOutput("SwerveStates/SetpointsOptimized",
    // optimizedSetpointStates);
  }

  public void runFieldOrientedDirection(Translation2d translation) {
    double currentDegrees = getRotation().getDegrees();
    double omegaOutput = fieldOrientedDirectionController.calculate(currentDegrees);

    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), omegaOutput, getRotation()));
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Gets drive speed. */
  public ChassisSpeeds getChassisSpeed() {
    return kinematics.toChassisSpeeds(
        modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  public void setTargetDirection(double degrees) {
    fieldOrientedDirectionController.setSetpoint(degrees);
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  // @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return pose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return gyroInputs.yawPosition;
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    this.pose = pose;
    poseEstimator.resetPosition(getRotation(), getWheelPositions(), pose);
  }

  // @AutoLogOutput(key = "Odometry/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  public void updateRobotPosition() {
    Pose2d kinematicPose = getPose();

    poseEstimator.update(getRotation(), getWheelPositions());

    Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
    int tags = LimelightHelpers.getRawFiducials("limelight").length;
    double distanceBetweenPoses = kinematicPose.getTranslation().getDistance(visionPose.getTranslation());

    
    if (tags > 0) {
      if (distanceBetweenPoses <= 0.5) {
        poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
      }
    }
    
    setPose(getEstimatedPose());
    
    Logger.recordOutput("Limelight/RobotPose", visionPose);
    Logger.recordOutput("Odometry/EstimatedPose", poseEstimator.getEstimatedPosition());
  }

}
