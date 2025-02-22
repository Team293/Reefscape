package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaepickup.AlgaeIntake;

public class AlgaeIntakeExtendPistons extends Command {
    private final AlgaeIntake algaeSolenoidRight;
    private final AlgaeIntake algaeSolenoidLeft;

    public AlgaeIntakeExtendPistons(AlgaeIntake algaeSolenoidLeft, AlgaeIntake algaeSolenoidRight) {
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

