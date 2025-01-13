package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        // ...
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    public void setSpeed(double speed);
}
