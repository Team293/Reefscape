package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.PositionVoltage;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        // ...
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}
    public void applyPosition(PositionVoltage request);
}
