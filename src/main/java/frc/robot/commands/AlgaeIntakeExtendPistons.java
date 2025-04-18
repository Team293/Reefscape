package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaePickup.AlgaePickup;

public class AlgaeIntakeExtendPistons extends Command {
    private final AlgaePickup algaePickup;

    public AlgaeIntakeExtendPistons(AlgaePickup algaePickup) {
        this.algaePickup = algaePickup;

        this.addRequirements(this.algaePickup);
    }

    @Override
    public void initialize() {
        algaePickup.extendAlagePickup();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

