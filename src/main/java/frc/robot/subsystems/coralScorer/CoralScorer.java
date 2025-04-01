package frc.robot.subsystems.coralScorer;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Thread.State;
import java.util.Currency;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.subsystems.algaePickup.ColorSensorIOInputsAutoLogged;
import frc.robot.subsystems.coralScorer.RightSightSensor;


public class CoralScorer extends SubsystemBase {
    public static enum States {
        INTAKE,
        HAS_PIECE,
        DROP,
        POINT_DOWN
    }

    private final Pneumatics pneumatics;

    private final CoralScorerIOTalonFX coralScorerMotor;
    private final CoralScorerIOInputsAutoLogged inputs = new CoralScorerIOInputsAutoLogged();
    private final ColorSensorIOInputsAutoLogged rightSightSensorInputs =
      new ColorSensorIOInputsAutoLogged();

    private DoubleSolenoid coralSolenoid;
    private final RightSightSensor proximitySensorIO;

    public static final double MAX_VELOCITY = 10.0d;
    public static final double TARGET_VELOCITY = 5.0d;
    public static final double HOLDING_VELOCITY = 1.0d;
    public static final double DROP_TIME_SEC = 0.4d;

    public static double previousVelocity = 0.0d;

    private boolean hasPiece = false;
    private CoralScorer.States state = States.INTAKE;
    private Timer stateTimer = new Timer();
    
    public CoralScorer(Pneumatics pneumatics) {
        this.pneumatics = pneumatics;
        
        coralSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
        coralScorerMotor = new CoralScorerIOTalonFX(0);
        proximitySensorIO = new RightSightSensor(0);

        stateTimer.start();
    }

    @Override
    public void periodic() {
        coralScorerMotor.updateInputs(inputs);
        proximitySensorIO.updateInputs(rightSightSensorInputs);
    
        SmartDashboard.putBoolean("HasPiece", hasPiece);
        SmartDashboard.putString("State", state.name());
    
        if (state == States.INTAKE) {
            pointUp();
        
            if (!hasPiece) {
                coralScorerMotor.setSpeed(-TARGET_VELOCITY);
                if (inputs.current > 4.5 && stateTimer.hasElapsed(0.2)) {
                    hasPiece = true;
                    setState(States.HAS_PIECE);
                }
            } else {
                coralScorerMotor.setSpeed(0);
            }
        } else if (state == States.HAS_PIECE) {
            if (!stateTimer.hasElapsed(0.4)) {
                coralScorerMotor.setSpeed(-TARGET_VELOCITY);
            } else {
                coralScorerMotor.setSpeed(0);
            }
        } else if (state == States.POINT_DOWN) {
            if (!stateTimer.hasElapsed(1)) {
                coralScorerMotor.setSpeed(-TARGET_VELOCITY);
            } else {
                coralScorerMotor.setSpeed(0.0);
            }
            pointDown();
        
        } else if (state == States.DROP) {
            if (!stateTimer.hasElapsed(DROP_TIME_SEC)) {
                coralScorerMotor.setSpeed(TARGET_VELOCITY);
            } else {
                if (stateTimer.hasElapsed(DROP_TIME_SEC + 0.4)) {    
                    hasPiece = false;
                    setState(States.INTAKE);
                } else {
                    pointUp();
                }
            }
        }
    }

    public boolean hasCoral() {
        // Updated when m_sensorIO.updateInputs(m_sensorInputs) happens in periodic
        // return (rightSightSensorInputs.IsCoralDetected);
        return hasPiece;
    }

    public Timer getStateTimer() {
        return stateTimer;
    }

    public States getState() {
        return state;
    }

    public void setState(States state) {
        if (this.state == States.DROP && !stateTimer.hasElapsed(DROP_TIME_SEC)) return;

        if (this.state != state) {
            stateTimer.restart();
            this.state = state;
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

    public void pointDown() {
        coralSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void disableCoralScorer() {
        coralScorerMotor.setSpeed(0.0);
    }

    public void forwardMotor() {
        coralScorerMotor.setSpeed(TARGET_VELOCITY);
    }

    public void reverseMotor() {
        coralScorerMotor.setSpeed(-TARGET_VELOCITY);
    }
}