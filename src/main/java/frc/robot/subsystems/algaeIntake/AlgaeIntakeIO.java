package frc.robot.subsystems.algaeIntake;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO {
    @AutoLog
    public static class AlgaePickupIOInputs {
        // ...
    }

    public default void updateInputs(AlgaePickupIOInputs inputs) {}
    public void setSpeed(double speed);
}
