package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOTalonFX implements ElevatorIO {
    private TalonFX elevatorMotor;
    private double m_gearRatio = 4/1; // change to the actual later

    public ElevatorIOTalonFX(int canID) {
        this.elevatorMotor = new TalonFX(canID);
        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = m_gearRatio;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set motor PID
        config.Slot0.kP = 0.11;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.462;
        config.Slot0.kS = 0.05;
        elevatorMotor.getConfigurator().apply(config);

        elevatorMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionValue = this.elevatorMotor.getPosition().getValueAsDouble();
    }

    public void applyPosition(PositionVoltage request) {
        elevatorMotor.setControl(request);
    }

    public void setBrakeMode(boolean brakeMode) {
        this.elevatorMotor.setNeutralMode(
            brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast
        );
    }
}
