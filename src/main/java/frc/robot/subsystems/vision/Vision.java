package frc.robot.subsystems.vision;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;

public class Vision extends SubsystemBase {
    private final String NAME = "limelight";

    private Drive drive;

    public Vision(Drive drive) {
        this.drive = drive;

        // LimelightHelpers.setCameraPose_RobotSpace(NAME, 
        //     0.5,    // Forward offset (meters)
        //     0.0,    // Side offset (meters)
        //     0.5,    // Height offset (meters)
        //     0.0,    // Roll (degrees)
        //     30.0,   // Pitch (degrees)
        //     0.0     // Yaw (degrees)
        // );
    }

    public void init() {

    }

    // public void periodic() {
    //     Pose2d botPose = LimelightHelpers.getBotPose2d_wpiBlue(NAME);

    //     Logger.recordOutput("vision/robotPose", botPose);
    // }

    // public void getPose()
    // {
 
    // }
}
