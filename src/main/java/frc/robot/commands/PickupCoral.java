package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralScorer.CoralScorer;
import frc.robot.subsystems.coralScorer.CoralScorer.States;

public class PickupCoral extends Command {
    private final CoralScorer coralScorer;

    public PickupCoral(CoralScorer coralScorer) {
        this.coralScorer = coralScorer;

    }

    @Override
    public void initialize() {
        coralScorer.setState(States.INTAKE);
    }

    @Override
    public boolean isFinished() {
        return coralScorer.hasCoral();
    }
}
