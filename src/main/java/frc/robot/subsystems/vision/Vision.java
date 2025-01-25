package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class Vision extends SubsystemBase {
    private final String NAME = "limelight";

    public Vision() {
        LimelightHelpers.setCameraPose_RobotSpace(NAME,
            0.2, // forward
            0.0, // side
            0.19, // up
            0.0, // roll
            0.0, // pitch
            0.0 // yaw
        );
    }

    public void updateRobotPose(SwerveDrivePoseEstimator estimator, double gyroRate) {
        boolean rejectOdometry = false;

        LimelightHelpers.SetRobotOrientation(
            NAME, 
            estimator.getEstimatedPosition().getRotation().getDegrees(),
            gyroRate,
            0,
            0,
            0,
            0
        );

        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(NAME);

        if (poseEstimate != null)
        {
            if (Math.abs(gyroRate) > 720) {
                rejectOdometry = true;
            }
    
            if (poseEstimate.tagCount == 0) {
                rejectOdometry = true;  
            }
    
            if (!rejectOdometry) {
                Pose2d visionPose = poseEstimate.pose;
    
                Logger.recordOutput("Limelight/EstimatedPose", visionPose);
    
                estimator.addVisionMeasurement(visionPose, poseEstimate.timestampSeconds);
            }
        }
    }
    
}
