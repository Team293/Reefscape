package frc.robot.subsystems.algaeknocker;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeKnockerIO {
    @AutoLog
    public static class AlgaeKnockerIOInputs {
        // ...
    }

    public default void updateInputs(AlgaeKnockerIOInputs inputs) {}
    public void setSpeed(double speed);
}
