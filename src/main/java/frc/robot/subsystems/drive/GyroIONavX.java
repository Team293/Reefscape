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

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** IO implementation for NavX */
public class GyroIONavX implements GyroIO {
  public final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  public static final boolean INVERTED = true;

  public GyroIONavX() {
    gyro.reset();
  }

  public void ResetYaw() {
    gyro.reset();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.realYawPosition = Rotation2d.fromDegrees(gyro.getYaw() * (INVERTED ? -1.0 : 1.0));
    inputs.yawPosition = inputs.realYawPosition.minus(inputs.yawOffset);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getRate());
    inputs.pose = gyro.getRotation3d();
  }
}
