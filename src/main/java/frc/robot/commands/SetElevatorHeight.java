package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class SetElevatorHeight extends Command {
    private final Elevator elevator;
    private final double targetHeight;
    private final double startingPosition;
    private final double perFrame;
    private final int totalFrames;
    
    private double currentSetHeight;
    private int frame = 0;

    public SetElevatorHeight(Elevator elevator, int level, int frames) {
        this.totalFrames = frames;
        this.elevator = elevator;
        this.targetHeight = Elevator.getTargetHeight(level - 1);
        this.startingPosition = elevator.getPosition();
        this.currentSetHeight = startingPosition;

        this.perFrame = (targetHeight - startingPosition) / frames;
    }

    @Override
    public void initialize() {
        elevator.setPosition(currentSetHeight);
    }

    @Override
    public void execute() {
        if (frame < totalFrames) {
            currentSetHeight += perFrame;
            frame++;
        }

        elevator.setPosition(currentSetHeight);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(elevator.getPosition() - targetHeight)) < 0.05;
    }
}
