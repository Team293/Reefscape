package frc.robot.subsystems.coralScorer;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralScorer extends SubsystemBase {
    private final Spark spark;
    private static final double PERCENT_OUTPUT = 0.5;
    
    public CoralScorer() {
        spark = new Spark(0);
    }

    @Override
    public void periodic() {
        
    }

    public void intakePiece() {
        spark.set(-PERCENT_OUTPUT);
    }

    public void outtakePiece() {
        spark.set(PERCENT_OUTPUT);
    }

    public void disableIntake() {
        spark.set(0);
    }
}