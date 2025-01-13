package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        // ...
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}
    public void setSpeed(double speed);
}
