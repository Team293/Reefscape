package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralScorer.CoralScorer;

public class PickupCoral extends Command {
    private final CoralScorer coralScorer;
    private final Timer timer;

    public PickupCoral(CoralScorer coralScorer) {
        this.coralScorer = coralScorer;
        this.timer = new Timer();

        this.addRequirements(coralScorer);
    }

    @Override
    public void initialize() {
        coralScorer.pointDown();
        coralScorer.reverseMotor();   
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
