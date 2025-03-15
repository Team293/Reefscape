package frc.robot.subsystems.coralScorer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.algaePickup.ColorSensorIO;
import frc.robot.subsystems.algaePickup.ColorSensorIO.ColorSensorIOInputs;

public class RightSightSensor implements ColorSensorIO  {
  private final DigitalInput m_sensor;

  public RightSightSensor(int channel) {
    m_sensor = new DigitalInput(channel);
  }

  @Override
  public void updateInputs(ColorSensorIOInputs inputs) {
    inputs.IsConnected = true;
    inputs.Proximity = 0;

    if (inputs.Proximity >= 200) {
      inputs.IsCoralDetected = true;
    }
  }
}
