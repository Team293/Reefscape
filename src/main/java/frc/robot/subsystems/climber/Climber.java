package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class Climber extends SubsystemBase {
    private static final double CLIMB_VELOCITY = 0.5;

    private final ClimberIOInputs inputs = new ClimberIOInputs();
    private final ClimberIOTalonFX climberMotor;
    private final DigitalInput bottomLimitSwitch;
    private final Timer timer;

    private boolean isClimbing;
    private boolean isClimbingUp;

    public Climber() {
        this.climberMotor = new ClimberIOTalonFX(12);
        this.bottomLimitSwitch = new DigitalInput(1); //TODO
        this.isClimbing = false;
        this.isClimbingUp = true;
        this.timer = new Timer();
    }

    @Override
    public void periodic() {
        climberMotor.updateInputs(inputs);

        if (isClimbing) {
            if (isClimbingUp && timer.hasElapsed(10))  { // Change time
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