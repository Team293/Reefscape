package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOInputsAutoLogged;
import frc.robot.util.LimelightHelpers;

public class Vision extends SubsystemBase {
    private static final String[] LIMELIGHT_NAMES = {
        "limelight-front",
        "limelight-back"
    };

    public Vision() {
        LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_NAMES[0],
            0.05, // forward
            0.0, // side
            0.19, // up
            0.0, // roll
            0.0, // pitch
            0.0 // yaw
        );

        LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_NAMES[1],
            -0.27, // forward
            0.0, // side
            0.24, // up
            0.0, // roll
            0.0, // pitch
            180.0 // yaw
        );
    }

    public void updateRobotPose(SwerveDrivePoseEstimator estimator, double gyroRate, GyroIOInputsAutoLogged gyroInputs) {
        if (Math.abs(gyroRate) > 720) {
            return;
        }
        
        for (String name : LIMELIGHT_NAMES) {
            setVisionPoseEstimation(name, estimator, gyroInputs);
        }
    }

    private void setVisionPoseEstimation(String limelightName, SwerveDrivePoseEstimator estimator, GyroIOInputsAutoLogged gyroInputs) {
        boolean rejectOdometry = false;

        double heading = gyroInputs.yawPosition.getDegrees();
        
        LimelightHelpers.SetRobotOrientation(
            limelightName, 
            heading,
            gyroInputs.yawVelocityRadPerSec * 180.0 / Math.PI,
            0,
            0,
            0,
            0
        );

        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);

        if (poseEstimate.tagCount == 0) {
            rejectOdometry = true;  
        }

        if (!rejectOdometry) {
            Pose2d visionPose = poseEstimate.pose;
            Pose2d currentPose = estimator.getEstimatedPosition();
            
            Logger.recordOutput("Limelight/EstimatedPose-" + limelightName, visionPose);

            // reject position greater than 1 meter apart from current
            if (currentPose.getTranslation().getDistance(visionPose.getTranslation()) > 100) {
                return;
            }


            estimator.addVisionMeasurement(visionPose, poseEstimate.timestampSeconds);
        }
    }
}
