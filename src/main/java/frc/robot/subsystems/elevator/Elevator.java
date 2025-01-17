package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIOTalonFX elevatorMotor;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    
    public Elevator() {
        elevatorMotor = new ElevatorIOTalonFX(0); // TODO: change to actual CAN ID
    }

    @Override
    public void periodic() {
        elevatorMotor.updateInputs(inputs);
    }

    public void enableElevator() {
        elevatorMotor.setSpeed(1.0);
    }

    public void disableElevator() {
        elevatorMotor.setSpeed(0.0);
    }
    
}
