package frc.robot.subsystems.algaepickup;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaePickup extends SubsystemBase {
    private final AlgaePickupIOTalonFX algaePickupMotor;
    private final AlgaePickupIOInputsAutoLogged inputs = new AlgaePickupIOInputsAutoLogged();
    
    public AlgaePickup() {
        algaePickupMotor = new AlgaePickupIOTalonFX(0); // TODO: change to actual CAN ID
    }

    @Override
    public void periodic() {
        algaePickupMotor.updateInputs(inputs);
    }

    public void enableAlgaePickup() {
        algaePickupMotor.setSpeed(1.0);
    }

    public void diableAlgaePickup() {
        algaePickupMotor.setSpeed(0.0);
    }
    
}
