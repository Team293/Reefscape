package frc.robot.subsystems.vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

import java.util.List;
import java.util.Map;

public class Vision extends SubsystemBase {
    private static final String[] LIMELIGHT_NAMES = {
        "limelight-front",
        "limelight-back"
    };

    private boolean isRunningPath= false;
    private Command runningCommand;

    public static enum AprilTagLineups {
        RED_6(new Pose2d()),
        RED_7(new Pose2d()),
        RED_8(new Pose2d()),
        RED_9(new Pose2d()),
        RED_10(new Pose2d()),
        RED_11(new Pose2d()),
        BLUE_16(new Pose2d()),
        BLUE_17(new Pose2d()),
        BLUE_19(new Pose2d()),
        BLUE_20(new Pose2d()),
        BLUE_21(new Pose2d()),
        BLUE_22(new Pose2d());

        private final Pose2d pose;

        AprilTagLineups(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

    private static final Rotation2d[] LIMELIGHT_YAW_OFFSETS = {
        Rotation2d.fromDegrees(180),
        Rotation2d.fromDegrees(0),
    };

    public Vision() {
        LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_NAMES[0],
            0.34925, // forward
            0.0, // side
            0.26035, // up
            180, // roll
            0, // pitch
            0 // yaw
        );

        LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_NAMES[1],
            0.05715, // forward
            0, // side
            0.88265, // up
            0, // roll
            0.0, // pitch
            180 // yaw
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
            gyroInputs.yawPosition.plus(yawOffset).getDegrees(),
            gyroInputs.yawVelocityRadPerSec / Math.PI * 180,
            0,
            0,
            0,
            0
        );

        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);

        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            rejectOdometry = true;  
        }

        if (!rejectOdometry) {
            Pose2d visionPose = poseEstimate.pose;
            Pose2d currentPose = estimator.getEstimatedPosition();
            
            Logger.recordOutput("Limelight/EstimatedPose-" + limelightName, visionPose);
            Logger.recordOutput("Limelight/RawEstimatedPose-" + limelightName, poseEstimate.pose);
            // reject position greater than 1 meter apart from current
            if (currentPose.getTranslation().getDistance(visionPose.getTranslation()) > 3) {
                return;
            }
            
            estimator.addVisionMeasurement(visionPose, poseEstimate.timestampSeconds);
        }
    }

    private void driveToPosition(Pose2d position) {
        PathConstraints constraints = PathConstraints.unlimitedConstraints(12); // TODO: lower if needed

        List<Waypoint> waypoints = List.of(
            new Waypoint(position.getTranslation(), position.getTranslation(), position.getTranslation())
        );

        PathPlannerPath path = new PathPlannerPath(
                waypoints, // waypoints, add more if needed
                constraints,
                null, // starting, not needed
                new GoalEndState(0.0, position.getRotation())
        );

        path.preventFlipping = true;

        Command followPath = AutoBuilder.followPath(path);

        Command cleanupCommand = Commands.run(() -> {
           this.isRunningPath = false;
           this.runningCommand = null;
        });

        followPath = followPath.andThen(cleanupCommand);

        followPath.schedule();

        isRunningPath = true;
    }

    public void interruptPath() {
        if (runningCommand != null) {
            runningCommand.cancel();
            runningCommand = null;
        }
        isRunningPath = false;
    }

    public void runPath(AprilTagLineups lineup) {
        if (isRunningPath) {
            return;
        }

        driveToPosition(lineup.getPose());
    }

    private AprilTagLineups getClosestTag(SwerveDrivePoseEstimator estimator) {
        Pose2d currentPose = estimator.getEstimatedPosition();
        double closestDistance = Double.MAX_VALUE;
        AprilTagLineups closestTag = null;

        for (AprilTagLineups tag : AprilTagLineups.values()) {
            double distance = currentPose.getTranslation().getDistance(tag.getPose().getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                closestTag = tag;
            }
        }

        return closestTag;
    }
}
