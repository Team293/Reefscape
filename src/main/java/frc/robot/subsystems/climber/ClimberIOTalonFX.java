package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;

public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX climberMotor;
    private final StatusSignal<AngularVelocity> currentVelocity;

    private static VelocityVoltage command = new VelocityVoltage(0.0d).withSlot(0);

    public ClimberIOTalonFX(int canId) {
        this.climberMotor = new TalonFX(canId);
        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = 1.0;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = 1; //TODO
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kV = 0.0; //TODO
        config.Slot0.kS = 0.0; //TODO

        climberMotor.getConfigurator().apply(config);

        currentVelocity = climberMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                currentVelocity
        );

        climberMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                currentVelocity
        );

        inputs.currentVelocity = currentVelocity.getValueAsDouble();
    }

    @Override
    public void setSpeed(double speed) {
        command.withVelocity(speed).withSlot(0);
        climberMotor.setControl(command);
    }
}
