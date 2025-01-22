package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIOTalonFX elevatorMotor;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private static final double MAX_POSITION = 1.0;
    private static final double MIN_POSITION = 0.0;
    
    public Elevator() {
        elevatorMotor = new ElevatorIOTalonFX(0); // TODO: change to actual CAN ID
    }

    @Override
    public void periodic() {
        elevatorMotor.updateInputs(inputs);
    }

    public void setPosition(double position) {
        if (position > MAX_POSITION) {
            position = MAX_POSITION;
        }

        if (position < MIN_POSITION) {
            position = MIN_POSITION;
        }

        PositionVoltage command = new PositionVoltage(0).withSlot(0);

        elevatorMotor.applyPosition(command.withPosition(position)); 
    }
}
