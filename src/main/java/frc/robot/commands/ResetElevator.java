package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ResetElevator extends Command {
    private final Elevator elevator;

    public ResetElevator(Elevator elevator) {
        this.elevator = elevator;

        this.addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.zero();
    }

    @Override
    public boolean isFinished() {
        return !elevator.getIsZeroing();
    }
}