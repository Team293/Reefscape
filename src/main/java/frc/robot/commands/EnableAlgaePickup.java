package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaePickup.AlgaePickup;

public class EnableAlgaePickup extends Command {
    private final AlgaePickup algaePickupMotor;
    private final Timer timer;

    public EnableAlgaePickup(AlgaePickup algaePickupMotor) {
        this.algaePickupMotor = algaePickupMotor;
        this.timer = new Timer();

        this.addRequirements(algaePickupMotor);
    }

    @Override
    public void initialize() {
        algaePickupMotor.enableAlgaeIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
