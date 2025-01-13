package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class Intake extends SubsystemBase {
    private final IntakeIOTalonFX intakeMotor;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    public Intake() {
        intakeMotor = new IntakeIOTalonFX(0); // TODO: change to actual CAN ID
    }

    @Override
    public void periodic() {
        intakeMotor.updateInputs(inputs);
    }

    public void enableIntake() {
        intakeMotor.setSpeed(1.0);
    }

    public void disableIntake() {
        intakeMotor.setSpeed(0.0);
    }
    
}
