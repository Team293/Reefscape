package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class SetElevator extends Command {
    private final double ELEVATOR_BUFFER_TIME = 0.75;

    private final Elevator elevator;
    private int level;
    private Timer buffer = new Timer();

    public SetElevator(Elevator elevator, int level) {
        this.elevator = elevator;
        this.level = level;
    }

    @Override
    public void initialize() {
        elevator.setPresetPos(level);
        buffer.start();
    }

    @Override
    public void execute() {
        if (!elevator.isAtTarget()) {
            buffer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        return buffer.hasElapsed(ELEVATOR_BUFFER_TIME);
    }
}
