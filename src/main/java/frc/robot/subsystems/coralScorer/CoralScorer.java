package frc.robot.subsystems.coralScorer;

import edu.wpi.first.wpilibj.Compressor;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CoralScorer extends SubsystemBase {
    private final CoralScorerIOTalonFX coralScorerMotor;
    private final CoralScorerIOInputsAutoLogged inputs = new CoralScorerIOInputsAutoLogged();
  
    // private Compressor compressor;
    private PneumaticHub hub;
    private DoubleSolenoid coralSolenoid;
    private Compressor compressor;
    private DigitalInput proximity;

    public static final double MAX_VELOCITY = 10.0d;
    public static double previousVelocity = 0.0d;

    public CoralScorer() {
        hub = new PneumaticHub(25);
        coralSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 4);
        compressor = new Compressor(25, PneumaticsModuleType.REVPH);
        compressor.enableAnalog(110, 120);
        coralScorerMotor = new CoralScorerIOTalonFX(3);
        proximity = new DigitalInput(0);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Compressor Pressure", compressor.getPressure());
        coralScorerMotor.updateInputs(inputs);

        if (proximity.get()) {
            coralScorerMotor.setSpeed(0);
        }
    }

    public void setVelocity(double targetVelocity) {
        double cappedTargetVelocity = Math.min(MAX_VELOCITY, Math.abs(targetVelocity));
        cappedTargetVelocity *= Math.signum(targetVelocity);

        coralScorerMotor.setSpeed(cappedTargetVelocity);
    }

    public void pointUp() {
        coralSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void reverseMotor() {
        coralScorerMotor.setSpeed(-5);
    }

    public void pointDown() {
        coralSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void forwardMotor() {
        coralScorerMotor.setSpeed(5); //TODO: Change later
    }

    public void disableCoralScorer() {
        coralScorerMotor.setSpeed(0.0);
    }
}