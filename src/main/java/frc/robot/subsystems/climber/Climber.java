package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;

public class Climber extends SubsystemBase {
    private static final double CLIMB_VELOCITY = 0.5;

    @AutoLog
    public static class ClimberIOInputs {
        public double currentVelocity = 0.0;
    }

    private final ClimberIOInputs inputs = new ClimberIOInputs();
    private final ClimberIOTalonFX climberMotor;
    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    private boolean isClimbing;
    private boolean isClimbingUp;

    public Climber() {
        this.climberMotor = new ClimberIOTalonFX(0); //TODO
        this.topLimitSwitch = new DigitalInput(0); //TODO
        this.bottomLimitSwitch = new DigitalInput(1); //TODO
        this.isClimbing = false;
        this.isClimbingUp = true;
    }

    @Override
    public void periodic() {
        climberMotor.updateInputs(inputs);

        if (isClimbing) {
            if (isClimbingUp && topLimitSwitch.get()) {
                // climber fully at top and ready to climb
                stopClimbing();
            } else if (!isClimbingUp && bottomLimitSwitch.get()) {
                // climber fully pulled down
                stopClimbing();
            } else {
                // we are in the middle of climbing
                climberMotor.setSpeed(isClimbingUp ? CLIMB_VELOCITY : -CLIMB_VELOCITY);
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
}