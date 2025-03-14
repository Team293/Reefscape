package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class Climber extends SubsystemBase {
    private static final double CLIMB_VELOCITY = 0.5;

    private final ClimberIOInputs inputs = new ClimberIOInputs();
    private final ClimberIOTalonFX climberMotor;

    private boolean isClimbing;
    private boolean isClimbingUp;

    public Climber() {
        this.climberMotor = new ClimberIOTalonFX(12);
        this.isClimbing = false;
        this.isClimbingUp = true;
    }

    @Override
    public void periodic() {
        climberMotor.updateInputs(inputs);

        if (isClimbing) {
            if (isClimbingUp && isClimberReset())  {
                // climber fully at top and ready to climb
                stopClimbing();
            } else if (isClimbing) {
                // the physical limit switch takes care of stopping the motor
                climberMotor.setSpeed(-CLIMB_VELOCITY);
            } else {
                // we are in the middle of resetting the climber to the up position
                climberMotor.setSpeed(CLIMB_VELOCITY);
            }
        } else {
            climberMotor.setSpeed(0);
        }
    }

    public void startClimbingUp() {
        isClimbing = true;
        isClimbingUp = true;
    }

    public void startClimbingDown() {
        isClimbing = true;
        isClimbingUp = false;
    }

    public void stopClimbing() {
        isClimbing = false;
        climberMotor.setSpeed(0);
    }

    private boolean isClimberReset() {
        if (isClimbing) {
            return inputs.supplyVoltage >= 10; //TODO: check the supply voltage
        } else {
            return false;
        }
    }
}