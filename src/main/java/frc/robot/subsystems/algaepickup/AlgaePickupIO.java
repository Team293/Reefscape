package frc.robot.subsystems.algaepickup;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaePickupIO {
    @AutoLog
    public static class AlgaePickupIOInputs {
        // ...
    }

    public default void updateInputs(AlgaePickupIOInputs inputs) {}
    public void setSpeed(double speed);
}
