package frc.robot.subsystems.coralScorer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.SubsystemDataProcessor.IODataRefresher;

public interface CoralScorerIO extends IODataRefresher {
    @AutoLog
    public static class CoralScorerIOInputs {
        public double speed;
        public double current;
    }

    public default void updateInputs(CoralScorerIOInputs inputs) {}
    public void setSpeed(double speed);
    @Override
    default void refreshData() {
        
    }
}

