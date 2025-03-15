package frc.robot.subsystems.algaePickup;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaePickup.AlgaePickupIOInputsAutoLogged;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaePickup extends SubsystemBase {
    private final AlgaePickupIOTalonFX algaePickupMotor;
    private DoubleSolenoid algaeSolenoidLeft;
    private DoubleSolenoid algaeSolenoidRight;

    
    private final AlgaePickupIOInputsAutoLogged inputs = new AlgaePickupIOInputsAutoLogged();

    public static final double MAX_VELOCITY = 20.0;
    
    public AlgaePickup() {
        algaePickupMotor = new AlgaePickupIOTalonFX(0);
       // algaeSolenoidLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH, 10, 8); //Change channels once testing
        algaeSolenoidRight = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5,7); //Change channels once testing    
    }

    @Override
    public void periodic() {
        algaePickupMotor.updateInputs(inputs);
        //proximitySensorIO.updateInputs(rightSightSensorInputs);
    }

    public void setVelocity(double targetVelocity) {
        double cappedTargetVelocity = Math.min(MAX_VELOCITY, Math.abs(targetVelocity));
        cappedTargetVelocity *= Math.signum(targetVelocity);

        algaePickupMotor.setSpeed(cappedTargetVelocity);
    }

    public void enableAlgaeIntake() {
        algaePickupMotor.setSpeed(1.0);
    }

    public void disableAlgaeIntake() {
        algaePickupMotor.setSpeed(0.0);
    }
    
    public void extendAlagePickup()
    {
       // algaeSolenoidLeft.set(DoubleSolenoid.Value.kForward);
        algaeSolenoidRight.set(DoubleSolenoid.Value.kForward);
    }
    
    public void retractAlgaePickup()
    {
       // algaeSolenoidLeft.set(DoubleSolenoid.Value.kReverse);
        algaeSolenoidRight.set(DoubleSolenoid.Value.kReverse);
    }

    /*public double detectedAlgaeForSeconds() {
        return rightSightSensorInputs.detectedForSeconds;
      }
    */
}
