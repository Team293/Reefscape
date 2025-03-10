package frc.robot.subsystems.coralScorer;

import org.littletonrobotics.junction.AutoLog;

public interface CoralScorerIO {
    @AutoLog
    public static class CoralScorerIOInputs {
        public double speed;
        public double current;
    }

    public default void updateInputs(CoralScorerIOInputs inputs) {}
    public void setSpeed(double speed);
}

