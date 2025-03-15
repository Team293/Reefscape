package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralScorer.CoralScorer;
import frc.robot.subsystems.coralScorer.CoralScorer.States;

public class SetCoralState extends Command {
    private final CoralScorer coralScorer;
    private final CoralScorer.States state;

    public SetCoralState(CoralScorer coralScorer, CoralScorer.States state) {
        this.coralScorer = coralScorer;
        this.state = state;

        this.addRequirements(coralScorer);
    }

    @Override
    public void initialize() {
        coralScorer.setState(state);
    }

    @Override
    public boolean isFinished() {
        if (this.state == States.DROP) {
            return coralScorer.getStateTimer().hasElapsed(0.2);
        }

        return true;
    }
}
