package frc.robot.subsystems.algaePickup;

import org.littletonrobotics.junction.AutoLog;

public interface ColorSensorIO {
  // @Autolog will automatically generate the specialized auto logging class in
  // build\generated\sources\annotationProcessor\java\main\frc\robot\subsystems\foo
  @AutoLog
  // The ColorSensorIOInputs class contains the variables that will be logged and shown within
  // AdvantageKit
  public static class ColorSensorIOInputs {
    // The variables that will be logged are declared here
    public boolean IsConnected;
    public double Blue;
    public int Proximity;
    public double MatchResultConfidence;
    public boolean IsCoralDetected;
    public boolean IsAlgaeDetected;
    public double detectedForSeconds;
  }

  // updateInputs should be called in periodic.
  // This is where the members above are updated.
  // This should be @Override'd with the actual implementation in the inheriting class.
  // This should not be updated here
  public default void updateInputs(ColorSensorIOInputs inputs) {}
}
