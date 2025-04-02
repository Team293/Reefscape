package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    private PneumaticHub hub = new PneumaticHub(1);

    public Pneumatics() {
        hub.enableCompressorAnalog(70, 80);
    // hub.disableCompressor();
    }

    public double getPressure() {
        return hub.getPressure(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Compressor Pressure", getPressure());
    }
}
