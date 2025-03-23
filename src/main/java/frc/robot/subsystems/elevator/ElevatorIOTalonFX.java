package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ElevatorIOTalonFX implements ElevatorIO {
    private TalonFX elevatorMotor;
    private StatusSignal<Angle> elevatorPosition;
    private StatusSignal<AngularVelocity> elevatorVelocity;
    private StatusSignal<ReverseLimitValue> limitSwitch;
    private double m_gearRatio = 11/10; 
    private double positionMeters;

    public ElevatorIOTalonFX(int canID) {
        this.elevatorMotor = new TalonFX(canID);
        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 30.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = m_gearRatio;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set motor PID
        config.Slot0.kP = 1.75;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.462;
        config.Slot0.kS = 0.05;
        config.Slot0.kG = 0.45;
        elevatorMotor.getConfigurator().apply(config);
        elevatorPosition = elevatorMotor.getPosition();
        elevatorVelocity = elevatorMotor.getVelocity();
        limitSwitch = elevatorMotor.getReverseLimit();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            elevatorPosition,
            elevatorVelocity,
            limitSwitch
        );

        elevatorMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            elevatorPosition,
            elevatorVelocity,
            limitSwitch
            );

        inputs.positionValue = elevatorPosition.getValueAsDouble();
        inputs.velocityValue = elevatorPosition.getValueAsDouble();
    }

    public void applyPosition(PositionVoltage request) {
        elevatorMotor.setControl(request);
    }

    public void setBrakeMode(boolean brakeMode) {
        this.elevatorMotor.setNeutralMode(
            brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast
        );
    }

    public void setZeroVoltage() {
        this.elevatorMotor.setControl(new VoltageOut(0));
    }

    public void setPosition(double position) {
        this.elevatorMotor.setPosition((position));
    }

    public void runVelocity(double velocity) {
        this.elevatorMotor.set(velocity);
    }

    public boolean isAtZero() {
        return this.limitSwitch.getValue() == ReverseLimitValue.ClosedToGround;
    }

    // public double encoderUnitsToMeters(double position){
    //      return (position/2048)*(1/m_gearRatio)*(1/(25/24))*(25 * 0.0049);
    // }
}
    