package frc.robot.subsystems.algaePickup;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.algaePickup.ColorSensorIO.ColorSensorIOInputs;

public class RightSightSensor implements ColorSensorIO  {
  private final DigitalInput m_sensor;
  private final Timer noteDetectionTimer = new Timer();

  public RightSightSensor(int channel) {
    m_sensor = new DigitalInput(channel);
    noteDetectionTimer.start();
  }

  @Override
  public void updateInputs(ColorSensorIOInputs inputs) {
    inputs.IsConnected = true;
    inputs.Blue = 0.0d;
    inputs.Proximity = 0;
    boolean signal = m_sensor.get();

    if (signal == false) {
      inputs.IsAlgaeDetected = true;
    } else {
      inputs.IsAlgaeDetected = false;
      noteDetectionTimer.reset();
      noteDetectionTimer.start();
    }

    inputs.detectedForSeconds = noteDetectionTimer.get();
  }
}
