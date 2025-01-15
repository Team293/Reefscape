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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SubsystemControl {
  private static final double DEADBAND = 0.1;

  private SubsystemControl() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude = Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          Rotation2d linearDirection;
          if (linearMagnitude < 0.01) {
            linearDirection = Rotation2d.fromDegrees(0);
          } else {
            linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          }
          
          double omega = omegaSupplier.getAsDouble();

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  drive.getRotation()));
        },
        drive);
  }

  /*
   * Field oriented direction
   */
  public static Command fieldOrientedRotation(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier targetDirection,
      DoubleSupplier rotationLeft,
      DoubleSupplier rotationRight) {
    return Commands.run(
        () -> {
          Translation2d translation =
              new Translation2d(
                  xSupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(),
                  ySupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec());

          double averageManualRotation = rotationLeft.getAsDouble() + rotationRight.getAsDouble();

          if (averageManualRotation != 0.0) {
            drive.setTargetDirection(drive.getRotation().getDegrees() + averageManualRotation);
          }

          if (targetDirection.getAsDouble() != -1.0) {
            drive.setTargetDirection(targetDirection.getAsDouble());
          }

          // Convert to field relative speeds & send command
          drive.runFieldOrientedDirection(translation);
        },
        drive);
  }

}
