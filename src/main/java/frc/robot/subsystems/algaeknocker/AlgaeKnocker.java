package frc.robot.subsystems.algaeknocker;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeKnocker extends SubsystemBase {
    private final Spark spark;
    private static final double PERCENT_OUTPUT = 0.5;
    
    public AlgaeKnocker() {
        spark = new Spark(0);
    }

    @Override
    public void periodic() {

    }

    public void enableAlgaeKnocker() {
        spark.set(PERCENT_OUTPUT);
    }

    public void disableAlgaeKnocker() {
        spark.set(0);
    }
}
