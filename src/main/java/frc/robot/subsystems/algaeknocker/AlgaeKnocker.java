package frc.robot.subsystems.algaeknocker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class AlgaeKnocker extends SubsystemBase {
    private final AlgaeKnockerIOTalonFX algaeKnockerMotor;
    private final AlgaeKnockerIOInputsAutoLogged inputs = new AlgaeKnockerIOInputsAutoLogged();
    
    public AlgaeKnocker() {
        algaeKnockerMotor = new AlgaeKnockerIOTalonFX(0); // TODO: change to actual CAN ID
    }

    @Override
    public void periodic() {
        algaeKnockerMotor.updateInputs(inputs);
    }

    public void enableAlgaeKnocker() {
        algaeKnockerMotor.setSpeed(1.0);
    }

    public void disableAlgaeKnocker() {
        algaeKnockerMotor.setSpeed(0.0);
    }
    
}
