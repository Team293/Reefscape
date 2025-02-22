package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;

public class EnableAlgaeIntake extends Command {
    private final AlgaeIntake algaeIntakeMotor;
    private final Timer timer;

    public EnableAlgaeIntake(AlgaeIntake algaeIntakeMotor) {
        this.algaeIntakeMotor = algaeIntakeMotor;
        this.timer = new Timer();

        this.addRequirements(algaeIntakeMotor);
    }

    @Override
    public void initialize() {
        algaeIntakeMotor.enableAlgaeIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
