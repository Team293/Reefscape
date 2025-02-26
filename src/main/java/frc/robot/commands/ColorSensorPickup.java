package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaePickup.AlgaePickup;

public class ColorSensorPickup extends Command {
  private final AlgaePickup algaePickup;

  public ColorSensorPickup(AlgaePickup algaePickup) {
    this.algaePickup = algaePickup;

    addRequirements(algaePickup);
  }

  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaePickup.enableAlgaeIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}