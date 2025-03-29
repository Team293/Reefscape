package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.robot.subsystems.pneumatics.Pneumatics;

public class Climber extends SubsystemBase {
    private static final double CLIMB_VELOCITY = 100;

    private final ClimberIOInputs inputs = new ClimberIOInputs();
    private final ClimberIOTalonFX climberMotor;
    private final Pneumatics pneumatics;
    private final DoubleSolenoid solenoid;

    private boolean isClimbing;
    private boolean isClimbingUp;

    public Climber(Pneumatics pneumatics) {
        this.pneumatics = pneumatics;
        this.solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
        this.climberMotor = new ClimberIOTalonFX(12);
        this.isClimbing = false;
        this.isClimbingUp = true;
        retractClimber();
    }

    @Override
    public void periodic() {
        climberMotor.updateInputs(inputs);
        extendClimber();

        if (isClimbing) {
            // if (isClimbingUp && isClimberReset())  {
            //     // climber fully at top and ready to climb
            //     stopClimbing();
            // } else
            if (isClimbingUp) {
                // the physical limit switch takes care of stopping the motor
                climberMotor.setSpeed(CLIMB_VELOCITY);
            } else {
                // we are in the middle of resetting the climber to the up position
                climberMotor.setSpeed(-CLIMB_VELOCITY);
            }
        } else {
            climberMotor.setSpeed(0);
        }
    }

    public void startClimbingUp() {
        isClimbing = true;
        isClimbingUp = true;
        Logger.recordOutput("Climber/IsClimbingUp", isClimbingUp);
    }

    public void startClimbingDown() {
        isClimbing = true;
        isClimbingUp = false;
    }

    public void stopClimbing() {
        isClimbing = false;
        climberMotor.setSpeed(0);
    }

    public void extendClimber() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retractClimber() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }
}