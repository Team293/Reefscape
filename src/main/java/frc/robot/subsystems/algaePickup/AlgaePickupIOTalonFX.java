package frc.robot.subsystems.algaePickup;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class AlgaePickupIOTalonFX implements AlgaePickupIO {
    private TalonFX algaePickupMotor;
    private double m_gearRatio = 4.0; // change to the actual later

    private static VelocityVoltage velocityVoltageCommand = new VelocityVoltage(0.0).withSlot(0);

    public AlgaePickupIOTalonFX(int canID) {
        this.algaePickupMotor = new TalonFX(canID);
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
        algaePickupMotor.getConfigurator().apply(config);

        algaePickupMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(AlgaePickupIOInputs inputs) {
        AlgaePickupIO.super.updateInputs(inputs);
    }

    @Override
    public void setSpeed(double speed) {
        velocityVoltageCommand.withVelocity(speed).withSlot(0); // Convert to motor rotations per second
        algaePickupMotor.setControl(velocityVoltageCommand);
    }
}
