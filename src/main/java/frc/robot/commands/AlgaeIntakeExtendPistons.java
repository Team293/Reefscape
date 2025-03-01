package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaePickup.AlgaePickup;

public class AlgaeIntakeExtendPistons extends Command {
    private final AlgaePickup algaeSolenoidRight;
    private final AlgaePickup algaeSolenoidLeft;

    public AlgaeIntakeExtendPistons(AlgaePickup algaeSolenoidLeft, AlgaePickup algaeSolenoidRight) {
        this.algaeSolenoidRight = algaeSolenoidRight;
        this.algaeSolenoidLeft = algaeSolenoidLeft;

        this.addRequirements(algaeSolenoidLeft);
        this.addRequirements(algaeSolenoidRight);
    }

    @Override
    public void initialize() {
        algaeSolenoidLeft.extendLeftSolenoid();
        algaeSolenoidRight.extendRightSolenoid();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

