package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;

public class ReverseAlgaePickup extends Command {
    private final AlgaeIntake algaeIntakeMotor;
    private final Timer timer;

    public ReverseAlgaePickup(AlgaeIntake algaeIntakeMotor) {
        this.algaeIntakeMotor = algaeIntakeMotor;
        this.timer = new Timer();

        this.addRequirements(algaeIntakeMotor);
    }

    @Override
    public void initialize() {
        algaeIntakeMotor.disableAlgaeIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
