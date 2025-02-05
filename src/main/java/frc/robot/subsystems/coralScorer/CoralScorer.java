package frc.robot.subsystems.coralScorer;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralScorer extends SubsystemBase {
    private final CoralScorerIOTalonFX CoralScorerMotor;
    private final CoralScorerIOInputsAutoLogged inputs = new CoralScorerIOInputsAutoLogged();
  
    private static final double PERCENT_OUTPUT = 0.5;
    private Compressor compressor;
    private PneumaticHub hub;
    private DoubleSolenoid coralSolenoid;
    public static final double MAX_VELOCITY = 10.0;

    public CoralScorer() {
        hub = new PneumaticHub(25);
        compressor = new Compressor(25, PneumaticsModuleType.REVPH);
        compressor.enableAnalog(110, 120);
        CoralScorerMotor = new CoralScorerIOTalonFX(4); // TODO: Change ID later

    }

    @Override
    public void periodic() {
          SmartDashboard.putNumber("Compresser Pressure", compressor.getPressure());
    }

    public void setVelocity(double targetVelocity) {
        double cappedTargetVelocity = Math.min(MAX_VELOCITY, Math.abs(targetVelocity));
        cappedTargetVelocity *= Math.signum(targetVelocity);

        CoralScorerMotor.setSpeed(cappedTargetVelocity);
    }

    public void intakePiece() {
        coralSolenoid.set(DoubleSolenoid.Value.kReverse);
        CoralScorerMotor.setSpeed(-1.0); //TODO Change later
    }

    public void outtakePiece() {
        coralSolenoid.set(DoubleSolenoid.Value.kForward);
        CoralScorerMotor.setSpeed(1.0); //TODO: Change later
    }

    public void disableCoralScorer() {
        CoralScorerMotor.setSpeed(0.0);
    }

    
}