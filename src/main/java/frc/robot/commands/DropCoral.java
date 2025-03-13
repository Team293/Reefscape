package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralScorer.CoralScorer;

public class DropCoral extends Command {
    private final CoralScorer coralScorer;
    private final Timer timer;

    public DropCoral(CoralScorer coralScorer) {
        this.coralScorer = coralScorer;
        this.timer = new Timer();

        this.addRequirements(coralScorer);
    }

    @Override
    public void initialize() {
        coralScorer.pointDown();
        timer.restart();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(0.5)) {
            coralScorer.forwardMotor();   
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1.0);
    }
}
