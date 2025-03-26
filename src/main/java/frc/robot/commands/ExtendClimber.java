package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class ExtendClimber extends Command {

    private final Climber climber;

    public ExtendClimber(Climber climber) {
        addRequirements(climber);

        this.climber = climber;
    }

    @Override
    public void initialize() {
        // Called just before this Command runs the first time
    }

    @Override
    public void execute() {
        climber.extendClimber();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
