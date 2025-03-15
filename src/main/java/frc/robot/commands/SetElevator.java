package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class SetElevator extends Command {
    private final Elevator elevator;
    private int level;

    public SetElevator(Elevator elevator, int level) {
        this.elevator = elevator;
        this.level = level;
    }

    @Override
    public void initialize() {
        elevator.setPresetPos(level);
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtTarget();
    }
}
