package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaePickup.AlgaePickup;

public class AlgaeIntakeRetractPistons extends Command {
    private final AlgaePickup algaePickup;

    public AlgaeIntakeRetractPistons(AlgaePickup algaePickup) {
        this.algaePickup = algaePickup;

        this.addRequirements(algaePickup);
    }

    @Override
    public void initialize() {
        algaePickup.retractAlgaePickup();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

