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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.constants.SDSMK4L1Constants;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final StatusSignal<Angle> drivePosition;
  // private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  // private final StatusSignal<Double> driveAppliedVolts;
  // private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  // private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  // private final StatusSignal<Double> turnAppliedVolts;
  // private final StatusSignal<Double> turnCurrent;

  private static final double DECELERATION_PER_SECOND = 1.0;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = SDSMK4L1Constants.driveGearRatio;
  // private static final double TURN_GEAR_RATIO = SDSMK4L1Constants.angleGearRatio;

  private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
  // private static final double WHEEL_CIRCUMFERENCE = 2.0 * WHEEL_RADIUS * Math.PI;
  private static final String CANBUS_ID = "rio";

  private final double absoluteEncoderOffset;

  private static VoltageOut voltageOutCommand = new VoltageOut(0.0);
  private static VelocityVoltage velocityVoltageCommand = new VelocityVoltage(0.0).withSlot(0);
  private static PositionVoltage positionVoltageCommand = new PositionVoltage(0.0).withSlot(0);

  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0: // Front Left
        driveTalon = new TalonFX(20);
        turnTalon = new TalonFX(21);
        cancoder = new CANcoder(22);
        absoluteEncoderOffset = -0.959d + 0.5d; // CANCoder rotations
        break;
      case 1: // Front Right
        driveTalon = new TalonFX(23);
        turnTalon = new TalonFX(24);
        cancoder = new CANcoder(25);
        absoluteEncoderOffset = -0.445d + 0.5d; // CANcoder rotations
        break;
      case 2: // Back Left
        driveTalon = new TalonFX(26);
        turnTalon = new TalonFX(27);
        cancoder = new CANcoder(28);
        absoluteEncoderOffset = -0.467d; // CANcoder rotations
        break;
      case 3: // Back Right
        driveTalon = new TalonFX(29);
        turnTalon = new TalonFX(30);
        cancoder = new CANcoder(31);
        absoluteEncoderOffset = -0.094d; // CANcoder rotations
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    // CANcoder config
    var canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.SensorDirection = SDSMK4L1Constants.canCoderSensorDirection;
    canCoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset + 0.25d;
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    cancoder.getConfigurator().apply(canCoderConfig);

    // Drive motor config
    driveTalon.getConfigurator().apply(getDriveConfig());
    driveTalon.clearStickyFaults();
    setDriveBrakeMode(true);
    drivePosition = driveTalon.getPosition();
    // drivePositionQueue =
    //     PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    // driveAppliedVolts = driveTalon.getMotorVoltage();
    // driveCurrent = driveTalon.getStatorCurrent();

    // Turn motor
    turnTalon.getConfigurator().apply(getTurnConfig());
    setTurnBrakeMode(true);
    turnAbsolutePosition = cancoder.getAbsolutePosition(); /* 0.0 <-> 1.0 in mechanism rotations */
    // unnecessary because position is set by the cancoder
    turnTalon.setPosition(turnAbsolutePosition.getValueAsDouble());
    turnPosition = turnTalon.getPosition(); // motor rotations
    // turnPositionQueue =
    //     PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    // turnAppliedVolts = turnTalon.getMotorVoltage();
    // turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        drivePosition,
        turnPosition,
        driveVelocity,
        // driveAppliedVolts,
        // driveCurrent,
        // turnAbsolutePosition,
        turnVelocity
        // turnAppliedVolts,
        // turnCurrent
        );
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        // driveAppliedVolts,
        // driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity
        // turnAppliedVolts,
        // turnCurrent
        );

    inputs.canCoderRotations = cancoder.getAbsolutePosition().getValueAsDouble();
    inputs.canCoderAngle = Units.rotationsToDegrees(inputs.canCoderRotations);

    inputs.drivePositionRotations = drivePosition.getValueAsDouble();
    inputs.driveVelocityRotationsPerSec = driveVelocity.getValueAsDouble();
    // inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    // inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnPositionAngle = inputs.turnPosition.getDegrees();
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    // inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    // inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    voltageOutCommand.withOutput(volts);
    driveTalon.setControl(voltageOutCommand);
  }

  @Override
  public void setDriveVelocityRPS(double velocityRPS) {
    // get 
    double currentVelocity = driveTalon.getVelocity().getValueAsDouble();

    double newVelocity = velocityRPS;
    // if setting velocity to zero, limit deceleration to 
    if (velocityRPS == 0 && Math.abs(velocityRPS - currentVelocity) > 0.3) {
      newVelocity = currentVelocity - Math.signum(currentVelocity) * (DECELERATION_PER_SECOND / 50.0);
    }

    velocityVoltageCommand.withVelocity(newVelocity).withSlot(0);
    driveTalon.setControl(velocityVoltageCommand);
  }

  @Override
  public void setTurnVoltage(double volts) {
    voltageOutCommand.withOutput(volts);
    turnTalon.setControl(voltageOutCommand);
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    positionVoltageCommand.withPosition(position.getRotations()).withSlot(0);
    turnTalon.setControl(positionVoltageCommand);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = SDSMK4L1Constants.driveMotorInvert;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = SDSMK4L1Constants.angleMotorInvert;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }

  private TalonFXConfiguration getDriveConfig() {
    /* Drive motor config */
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted = SDSMK4L1Constants.driveMotorInvert;

    driveConfig.Slot0.kP = 0.15;
    driveConfig.Slot0.kI = 0.0;
    driveConfig.Slot0.kD = 0.005;
    driveConfig.Slot0.kS = 0.15;
    driveConfig.Slot0.kV = 0.93;

    driveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

    return driveConfig;
  }

  private TalonFXConfiguration getTurnConfig() {
    /* Turn motor config */
    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;

    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.MotorOutput.Inverted = SDSMK4L1Constants.angleMotorInvert;

    turnConfig.Slot0.kP = 18.0;
    turnConfig.Slot0.kI = 0.0;
    turnConfig.Slot0.kD = 0.0;
    turnConfig.Slot0.kS = 0.28;
    turnConfig.Slot0.kV = 1.54;

    // turnConfig.Feedback.SensorToMechanismRatio = TURN_GEAR_RATIO;
    turnConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    turnConfig.Feedback.SensorToMechanismRatio = 1.0;
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    return turnConfig;
  }
}
