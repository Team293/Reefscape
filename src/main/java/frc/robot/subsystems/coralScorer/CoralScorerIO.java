package frc.robot.subsystems.coralScorer;

import org.littletonrobotics.junction.AutoLog;

public interface CoralScorerIO {
    @AutoLog
    public static class CoralScorerIOInputs {
        // ...
    }

    public default void updateInputs(CoralScorerIOInputs inputs) {}
    public void setSpeed(double speed);
}

