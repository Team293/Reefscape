package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class Vision extends SubsystemBase {
    private static final String[] LIMELIGHT_NAMES = {
        "front",
        "back"
    };

    public Vision() {
        for (String name : LIMELIGHT_NAMES) {
            setupLimelight(name);
        }
    }

    private void setupLimelight(String limelightName) {
        LimelightHelpers.setCameraPose_RobotSpace(limelightName,
            0.2, // forward
            0.0, // side
            0.19, // up
            0.0, // roll
            0.0, // pitch
            0.0 // yaw
        );
    }

    public void updateRobotPose(SwerveDrivePoseEstimator estimator, double gyroRate) {
        if (Math.abs(gyroRate) > 720) {
            return;
        }
        
        for (String name : LIMELIGHT_NAMES) {
            setVisionPoseEstimation(name, estimator);
        }
    }

    private void setVisionPoseEstimation(String limelightName, SwerveDrivePoseEstimator estimator) {
        boolean rejectOdometry = false;

        LimelightHelpers.SetRobotOrientation(
            limelightName, 
            estimator.getEstimatedPosition().getRotation().getDegrees(),
            0,
            0,
            0,
            0,
            0
        );

        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (poseEstimate.tagCount == 0) {
            rejectOdometry = true;  
        }

        if (!rejectOdometry) {
            Pose2d visionPose = poseEstimate.pose;
            Pose2d currentPose = estimator.getEstimatedPosition();

            // reject position greater than 1 meter apart from current
            if (currentPose.getTranslation().getDistance(visionPose.getTranslation()) > 1) {
                return;
            }

            Logger.recordOutput("Limelight/EstimatedPose-" + limelightName, visionPose);

            estimator.addVisionMeasurement(visionPose, poseEstimate.timestampSeconds);
        }
    }
}
