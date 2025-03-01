package frc.robot.subsystems.algaePickup;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensorIORevV3 implements ColorSensorIO {
    private final I2C.Port m_i2cPort;
    private final ColorSensorV3 m_colorSensor;
    private final Color m_noteColor = new Color(0.520, 0.378, 0.103); // todo Is this correct?
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Timer noteDetectionTimer = new Timer();
  
    public ColorSensorIORevV3() {
      m_i2cPort = I2C.Port.kMXP;
      m_colorSensor = new ColorSensorV3(m_i2cPort);
  
      m_colorMatcher.setConfidenceThreshold(0.95);
      m_colorMatcher.addColorMatch(m_noteColor);
      noteDetectionTimer.start();
    }
  
    @Override
    public void updateInputs(ColorSensorIOInputs inputs) {
      inputs.IsConnected = true;
      Color detectedColor = m_colorSensor.getColor();
      inputs.Blue = detectedColor.blue;
      inputs.Proximity = m_colorSensor.getProximity();
  
      /** Run the color match algorithm on our detected color */
      // ColorMatchResult match = m_colorMatcher.matchColor(detectedColor);
  
      if (inputs.Proximity >= 200) {
        inputs.IsAlgaeDetected = true;
      } else {
        inputs.IsAlgaeDetected = false;
        noteDetectionTimer.reset();
        noteDetectionTimer.start();
      }
  
      inputs.detectedForSeconds = noteDetectionTimer.get();
    }
  }