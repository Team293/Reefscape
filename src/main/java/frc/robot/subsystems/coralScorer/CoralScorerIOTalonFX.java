package frc.robot.subsystems.coralScorer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class CoralScorerIOTalonFX implements CoralScorerIO {
    private TalonFX CoralScorerMotor;
    private double m_gearRatio = 1.0; // change to the actual later

    private StatusSignal<Angle> position;
    private StatusSignal<AngularVelocity> velocity;

    private static VelocityVoltage velocityVoltageCommand = new VelocityVoltage(0.0).withSlot(0);

    public CoralScorerIOTalonFX(int canID) {
        this.CoralScorerMotor = new TalonFX(canID);
        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = m_gearRatio;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set motor PID, TODO: Set PID
        config.Slot0.kP = 0.11;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.462;
        config.Slot0.kS = 0.05;
        CoralScorerMotor.getConfigurator().apply(config);

        position = CoralScorerMotor.getPosition();
        velocity = CoralScorerMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            position,
            velocity
        );

        CoralScorerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(CoralScorerIOInputs inputs) {
        BaseStatusSignal.refreshAll(velocity);
        inputs.speed = velocity.getAsDouble();
        CoralScorerIO.super.updateInputs(inputs);
    }

    @Override
    public void setSpeed(double speed) {
        velocityVoltageCommand.withVelocity(speed).withSlot(0); // Convert to motor rotations per second
        CoralScorerMotor.setControl(velocityVoltageCommand);
    }
}
