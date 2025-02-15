package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeknocker.AlgaeKnocker;

// For L2 algae
public class DisableAlgaeKnocker extends Command {
    private final AlgaeKnocker algaeKnocker;
    private final Timer timer;

    public DisableAlgaeKnocker(AlgaeKnocker algaeKnocker) {
        this.algaeKnocker = algaeKnocker;
        this.timer = new Timer();

        this.addRequirements(algaeKnocker);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        algaeKnocker.disableAlgaeKnocker();   
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1.0);
    }
}