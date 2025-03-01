package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
import frc.robot.util.LimelightHelpers;

public class Vision extends SubsystemBase {
    private static final String[] LIMELIGHT_NAMES = {
        "limelight-front",
        "limelight-back"
    };

    private static final Rotation2d[] LIMELIGHT_YAW_OFFSETS = {
        Rotation2d.fromDegrees(180.0),
        Rotation2d.fromDegrees(180.0)
    };

    public Vision() {
        LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_NAMES[0],
            0.05, // forward
            0.0, // side
            0.19, // up
            0.0, // roll
            16, // pitch
            180 // yaw
        );

        LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_NAMES[1],
            0.34, // forward
            0.0, // side
            0.24, // up
            0.0, // roll
            0.0, // pitch
            0 // yaw
        );

        // force limelights to use roboRIO for gyro
        for (String name : LIMELIGHT_NAMES) {
            LimelightHelpers.SetIMUMode(name, 0);
        }
    }

    public void updateRobotPose(SwerveDrivePoseEstimator estimator, double gyroRate, GyroIOInputsAutoLogged gyroInputs) {
        if (Math.abs(gyroRate) > 720) {
            return;
        }
        
        int i = 0;
        for (String name : LIMELIGHT_NAMES) {
            setVisionPoseEstimation(name, estimator, gyroInputs, LIMELIGHT_YAW_OFFSETS[i]);
            i++;
        }
    }

    private void setVisionPoseEstimation(String limelightName, SwerveDrivePoseEstimator estimator, GyroIOInputs gyroInputs, Rotation2d yawOffset) {
        boolean rejectOdometry = false;

        LimelightHelpers.SetRobotOrientation(
            limelightName, 
            gyroInputs.yawPosition.getDegrees(),
            gyroInputs.yawVelocityRadPerSec / Math.PI * 180,
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
            Pose2d visionPose = new Pose2d(poseEstimate.pose.getTranslation(), poseEstimate.pose.getRotation().plus(yawOffset));
            Pose2d currentPose = estimator.getEstimatedPosition();
            
            Logger.recordOutput("Limelight/EstimatedPose-" + limelightName, visionPose);

            // reject position greater than 1 meter apart from current
            if (currentPose.getTranslation().getDistance(visionPose.getTranslation()) > 3) {
                return;
            }
            
            estimator.addVisionMeasurement(visionPose, poseEstimate.timestampSeconds);
        }
    }
}
