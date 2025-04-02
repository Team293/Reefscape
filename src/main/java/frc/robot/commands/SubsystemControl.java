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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.SpikeController;
import frc.robot.subsystems.coralScorer.CoralScorer;
import frc.robot.subsystems.coralScorer.CoralScorer.States;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.CoralLineup;

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

  public static Command joystickDrive(
    Drive drive,
    Vision vision,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier omegaSupplier,
    DoubleSupplier strafeLeft,
    DoubleSupplier strafeRight,
    BooleanSupplier selfDrivingLeft,
    BooleanSupplier selfDrivingRight
    ) {
  return Commands.run(
      () -> {
        // vision.interruptPath(); // allow driver to override control at all times

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

        if (Math.abs(strafe) > 0.001) {
          xTranslation = Math.sin(-drive.getRotation().getRadians()) * strafe * 0.1;
          yTranslation = Math.cos(-drive.getRotation().getRadians()) * strafe * 0.1;
        } else {
          xTranslation = linearVelocity.getX();
          yTranslation = linearVelocity.getY();
        }

        if (selfDrivingLeft.getAsBoolean()) {
          drive.setTargetPose(vision.closestTargetPose(drive.getPose(), CoralLineup.LEFT));
          drive.driveToTargetPose();
          return;
        } else if (selfDrivingRight.getAsBoolean()) {
          drive.setTargetPose(vision.closestTargetPose(drive.getPose(), CoralLineup.RIGHT));
          drive.driveToTargetPose();
          return;
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
      // DoubleSupplier elevatorPercentage,
      SpikeController controller
    )
    {
    return Commands.run(() -> {

      // if (Math.abs(elevatorPercentage.getAsDouble()) > 0.05) {
      //   elevator.changePosition(-elevatorPercentage.getAsDouble());
      // }

      if (controller.x().getAsBoolean()) {
        elevator.setPresetPos(1); // L2
      } else if (controller.a().getAsBoolean()) {
        elevator.setPresetPos(2); // L3
      } else if (controller.b().getAsBoolean()) {
        elevator.setPresetPos(3); // L4
      // } else if (controller.y().getAsBoolean()) {
      //   elevator.setPresetPos(4); // coral station
      } else if (controller.leftTrigger().getAsBoolean()) {
        elevator.zero();
      }
    }, elevator);
  }

  public static Command coralControl(
    CoralScorer coralScorer,
    DoubleSupplier updown,
    BooleanSupplier output,
    BooleanSupplier pointDown
  ) {
    return Commands.run(() -> {
      if (coralScorer.hasCoral()) {
        if (output.getAsBoolean()) {
          coralScorer.setState(States.DROP);
        } else {
          if (pointDown.getAsBoolean()) {
            coralScorer.setState(States.POINT_DOWN);
          }
        }
      } else {
        if (coralScorer.getState() != States.HAS_PIECE) {
          coralScorer.setState(States.INTAKE);
        }
      }
    }, coralScorer);
  }

  public static Command climb(
    Climber climber,
    BooleanSupplier up,
    BooleanSupplier down,
    BooleanSupplier extendClimber
  ) {
      return Commands.run(() -> {
        if (extendClimber.getAsBoolean()) {
          climber.extendClimber();
        }

          if (up.getAsBoolean()) {
              climber.startClimbingUp();
          } else if (down.getAsBoolean()) {
              climber.startClimbingDown();
          } else {
              climber.stopClimbing();
          }
      }, climber);
  }
}
