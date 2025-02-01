package frc.robot.subsystems.algaepickup;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaePickup extends SubsystemBase {
    private final AlgaePickupIOTalonFX algaePickupMotor;
    private final AlgaePickupIOInputsAutoLogged inputs = new AlgaePickupIOInputsAutoLogged();

    public static final double MAX_VELOCITY = 20.0;
    
    public AlgaePickup() {
        algaePickupMotor = new AlgaePickupIOTalonFX(1);
    }

    @Override
    public void periodic() {
        algaePickupMotor.updateInputs(inputs);
    }

    public void setVelocity(double targetVelocity) {
        double cappedTargetVelocity = Math.min(MAX_VELOCITY, Math.abs(targetVelocity));
        cappedTargetVelocity *= Math.signum(targetVelocity);

        algaePickupMotor.setSpeed(cappedTargetVelocity);
    }

    public void enableAlgaePickup() {
        algaePickupMotor.setSpeed(1.0);
    }

    public void diableAlgaePickup() {
        algaePickupMotor.setSpeed(0.0);
    }
    
}
