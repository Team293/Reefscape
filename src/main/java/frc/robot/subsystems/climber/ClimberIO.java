package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double currentVelocity = 0.0d;
        public double supplyVoltage = 0.0d;
    }

    void updateInputs(ClimberIOInputs inputs);
    void setSpeed(double speed);
}
