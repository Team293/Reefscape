package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralScorer.CoralScorer;

public class CoralBeamBreak extends Command {
  private final CoralScorer coralScorer;

  public CoralBeamBreak(CoralScorer coralScorer) {
    this.coralScorer = coralScorer;

    addRequirements(coralScorer);
  }

  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralScorer.disableCoralScorer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}