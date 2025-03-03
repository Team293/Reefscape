package frc.robot.subsystems.coralScorer;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.subsystems.algaePickup.ColorSensorIOInputsAutoLogged;
import frc.robot.subsystems.coralScorer.RightSightSensor;


public class CoralScorer extends SubsystemBase {
    private final Pneumatics pneumatics;

    private final CoralScorerIOTalonFX coralScorerMotor;
    private final CoralScorerIOInputsAutoLogged inputs = new CoralScorerIOInputsAutoLogged();
    private final ColorSensorIOInputsAutoLogged rightSightSensorInputs =
      new ColorSensorIOInputsAutoLogged();

    private DoubleSolenoid coralSolenoid;
    private final RightSightSensor proximitySensorIO;


    public static final double MAX_VELOCITY = 10.0d;
    public static double previousVelocity = 0.0d;

    public CoralScorer(Pneumatics pneumatics) {
        this.pneumatics = pneumatics;
        
        coralSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 4);
        coralScorerMotor = new CoralScorerIOTalonFX(3);
        proximitySensorIO = new RightSightSensor(0);
    }

    @Override
    public void periodic() {
       // SmartDashboard.putNumber("Compressor Pressure", compressor.getPressure());
        coralScorerMotor.updateInputs(inputs);
        proximitySensorIO.updateInputs(rightSightSensorInputs);
    }

    public boolean isCoralDetected() {
        // Updated when m_sensorIO.updateInputs(m_sensorInputs) happens in periodic
        return (rightSightSensorInputs.IsCoralDetected);
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