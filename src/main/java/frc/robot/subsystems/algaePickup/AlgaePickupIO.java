package frc.robot.subsystems.algaePickup;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaePickupIO {
    @AutoLog
    public static class AlgaePickupIOInputs {
        // ...
    }

    public default void updateInputs(AlgaePickupIOInputs inputs) {}
    public void setSpeed(double speed);
}
