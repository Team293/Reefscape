package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double currentVelocity = 0.0d;
    }

    public void updateInputs(ClimberIOInputs inputs);
    public void setSpeed(double speed);
}
