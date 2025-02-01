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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralScorer.CoralScorer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.lib.SpikeController;
import frc.robot.subsystems.algaepickup.AlgaePickup;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SubsystemControl {

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

  public static Command joystickDrive(
    Drive drive,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier omegaSupplier,
    DoubleSupplier strafeLeft,
    DoubleSupplier strafeRight) {
  return Commands.run(
      () -> {
        double strafe = strafeLeft.getAsDouble() - strafeRight.getAsDouble();

        double xTranslation = xSupplier.getAsDouble();
        double yTranslation = ySupplier.getAsDouble();

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

        if (Math.abs(strafe) > 0.05) {
          xTranslation = Math.sin(-drive.getRotation().getRadians()) * strafe;
          yTranslation = Math.cos(-drive.getRotation().getRadians()) * strafe;
        } else {
          xTranslation = linearVelocity.getX();
          yTranslation = linearVelocity.getY();
        }

        // Convert to field relative speeds & send command
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xTranslation * drive.getMaxLinearSpeedMetersPerSec(),
                yTranslation * drive.getMaxLinearSpeedMetersPerSec(),
                omega * drive.getMaxAngularSpeedRadPerSec(),
                drive.getRotation()));
      },
      drive);
  }

  public static Command elevatorControl(
    Elevator elevator,
    DoubleSupplier elevatorPercentage,
    BooleanSupplier resetElevator,
    SpikeController controller
  ) {
    return Commands.run(() -> {
      if (resetElevator.getAsBoolean()) {
        elevator.calculateOffset();
      }

      if (Math.abs(elevatorPercentage.getAsDouble()) > 0.05) {
        elevator.changePosition(-elevatorPercentage.getAsDouble());
      }

      if (controller.a().getAsBoolean()) {
        elevator.setPresetPos(0);
      } else if (controller.b().getAsBoolean()) {
        elevator.setPresetPos(1);
      } else if (controller.y().getAsBoolean()) {
        elevator.setPresetPos(2);
      } else if (controller.x().getAsBoolean()) {
        elevator.setPresetPos(3);
      }
      
    }, elevator);
  }

  public static Command coralControl(
    CoralScorer coralScorer,
    BooleanSupplier intake,
    BooleanSupplier output
  ) {
    return Commands.run(() -> {
      if (intake.getAsBoolean()) {
        coralScorer.intakePiece();
      } else if (output.getAsBoolean()) {
        coralScorer.outtakePiece();
      } else {
        coralScorer.disableIntake();
      }
    }, coralScorer);
  }

  public static Command alagaePickup(
    AlgaePickup algaePickup,
    DoubleSupplier speed
  )
  {
    return Commands.run(() -> {
      algaePickup.setVelocity(speed.getAsDouble() * AlgaePickup.MAX_VELOCITY);
    }, algaePickup);
  }

  // public static Command limelightDrive(
  //     Drive drive,
  //     Vision vision,
  //     DoubleSupplier xSupplier,
  //     DoubleSupplier ySupplier,
  //     DoubleSupplier omegaSupplier) {

  //   return Commands.run(
  //       () -> {
  //         // Apply deadband
  //         double linearMagnitude =
  //             MathUtil.applyDeadband(
  //                 Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
  //         Rotation2d linearDirection =
  //             new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
  //         double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

  //         // Square values
  //         linearMagnitude = linearMagnitude * linearMagnitude;
  //         omega = Math.copySign(omega * omega, omega);

  //         // Calcaulate new linear velocity
  //         Translation2d linearVelocity =
  //             new Pose2d(new Translation2d(), linearDirection)
  //                 .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
  //                 .getTranslation();

  //         if (omega != 0.0d) { // Check if the driver isnt trying to turn
  //           vision.resetError();
  //         } else if ((omega == 0.0) && (vision.seesTarget())) {
  //           // Get tX from the vision subsystem. tX is "demand"
  //           omega = -vision.getDesiredAngle();
  //         }

  //         drive.runVelocity(
  //             // Convert to field relative speeds & send command
  //             ChassisSpeeds.fromFieldRelativeSpeeds(
  //                 linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
  //                 linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
  //                 omega * drive.getMaxAngularSpeedRadPerSec(),
  //                 drive.getRotation()));
  //       },
  //       drive);
  // }
}
