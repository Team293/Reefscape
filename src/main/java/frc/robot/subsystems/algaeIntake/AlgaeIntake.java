package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaepickup.AlgaePickupIOInputsAutoLogged;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class AlgaeIntake extends SubsystemBase {
    private final AlgaeIntakeIOTalonFX algaeIntakeMotor;
    private PneumaticHub hub;
    private Compressor compressor;
    private DoubleSolenoid algaeSolenoidLeft;
    private DoubleSolenoid algaeSolenoidRight;

    
    private final AlgaePickupIOInputsAutoLogged inputs = new AlgaePickupIOInputsAutoLogged();

    public static final double MAX_VELOCITY = 20.0;
    
    public AlgaeIntake() {
        algaeIntakeMotor = new AlgaeIntakeIOTalonFX(1);
        hub = new PneumaticHub(25);
        algaeSolenoidLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 5); //Change channels once testing
        algaeSolenoidRight = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 6); //Change channels once testing      
        compressor = new Compressor(25, PneumaticsModuleType.REVPH);
        compressor.enableAnalog(110, 120);
    }

    @Override
    public void periodic() {
        algaeIntakeMotor.updateInputs(inputs);
    }

    public void setVelocity(double targetVelocity) {
        double cappedTargetVelocity = Math.min(MAX_VELOCITY, Math.abs(targetVelocity));
        cappedTargetVelocity *= Math.signum(targetVelocity);

        algaeIntakeMotor.setSpeed(cappedTargetVelocity);
    }

    public void enableAlgaeIntake() {
        algaeIntakeMotor.setSpeed(1.0);
    }

    public void disableAlgaeIntake() {
        algaeIntakeMotor.setSpeed(0.0);
    }

    public void extendRightSolenoid()
    {
        algaeSolenoidRight.set(DoubleSolenoid.Value.kForward);
    }

    public void extendLeftSolenoid()
    {
        algaeSolenoidLeft.set(DoubleSolenoid.Value.kForward);
    }

    public void retractLeftSolenoid()
    {
        algaeSolenoidLeft.set(DoubleSolenoid.Value.kReverse);
    }
    
    public void retractRightSolenoid()
    {
        algaeSolenoidRight.set(DoubleSolenoid.Value.kReverse);
    }
}
