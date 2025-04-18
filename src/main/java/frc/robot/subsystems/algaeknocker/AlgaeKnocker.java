package frc.robot.subsystems.algaeknocker;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.pneumatics.Pneumatics;

public class AlgaeKnocker extends SubsystemBase {
    private final TalonSRX talonSRX;
    // private DoubleSolenoid algaeKnocker;
    private static final double PERCENT_OUTPUT = 1.0;
    private final Pneumatics pneumatics;

    public AlgaeKnocker(Pneumatics pneumatics) {
        this.pneumatics = pneumatics;
        
        talonSRX = new TalonSRX(4);
        // algaeKnocker = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0,1); //Change channels once testing    
    }

    @Override
    public void periodic() {

    }

    public void forwardAlgaeKnockerMotor() {
        talonSRX.set(ControlMode.PercentOutput, PERCENT_OUTPUT);    
    }

    public void disableAlgaeKnockerMotor()
    {
        talonSRX.set(ControlMode.PercentOutput, 0);    
    }

    public void reverseAlgaeKnockerMotor() {
        talonSRX.set(ControlMode.PercentOutput, -PERCENT_OUTPUT);    
    }

    public void extendAlgaeKnocker() {
        // algaeKnocker.set(DoubleSolenoid.Value.kReverse);
    }

    public void retractAlgaeKnocker() {
        // algaeKnocker.set(DoubleSolenoid.Value.kForward);
    }
}
