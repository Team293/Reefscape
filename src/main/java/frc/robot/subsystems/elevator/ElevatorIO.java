package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.PositionVoltage;

import frc.robot.util.SubsystemDataProcessor.IODataRefresher;

public interface ElevatorIO extends IODataRefresher {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionValue = 0.0d;
        public double velocityValue = 0.0d;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}
    public void applyPosition(PositionVoltage request);
    
    @Override
    default void refreshData() {
        
    }
}
