package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeknocker.AlgaeKnocker;

// For L3 algae
public class EnableAlgaeKnocker extends Command {
    private final AlgaeKnocker algaeKnocker;
    private final Timer timer;

    public EnableAlgaeKnocker(AlgaeKnocker algaeKnocker) {
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
        algaeKnocker.forwardAlgaeKnockerMotor();
        algaeKnocker.extendAlgaeKnocker();   
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1.0);
    }
}