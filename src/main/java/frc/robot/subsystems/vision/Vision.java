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
        // "limelight-left"
        "limelight-right"
    };

    private boolean isRunningPath= false;
    private Command runningCommand;

    public static enum AprilTagLineups {
        // RED_6(new Pose2d()),
        // RED_7(new Pose2d()),
        // RED_8(new Pose2d()),
        // RED_9(new Pose2d()),
        RED_10(new Pose2d(5.75, 4.00, Rotation2d.fromDegrees(0 + 180))); // reef far
        // RED_11(new Pose2d()),
        // BLUE_16(new Pose2d()),
        // BLUE_17(new Pose2d()),
        // BLUE_19(new Pose2d()),
        // BLUE_20(new Pose2d()),
        // BLUE_21(new Pose2d()),
        // BLUE_22(new Pose2d());

        private final Pose2d pose;

        AprilTagLineups(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

    public static enum CoralLineup {
        LEFT(0.0d),
        RIGHT(0.0d);

        private final double xTranslation;

        CoralLineup(double xTranslation) {
            this.xTranslation = xTranslation;
        }

        public double getXTranslation() {
            return xTranslation;
        }
    }

    private static final Rotation2d[] LIMELIGHT_YAW_OFFSETS = {
        // Rotation2d.fromDegrees(22.48),
        Rotation2d.fromDegrees(26.36),
    };

    public Vision() {
        // LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_NAMES[0],
        //     0.2477516, // forward
        //     -0.23495, // side
        //     0.3302, // up
        //     180, // roll
        //     0, // pitch
        //     -90 + 22.48 // yaw
        // );

        LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_NAMES[0],
            0.24765, // forward
            0.2159, // side
            0.3302, // up
            180, // roll
            0.0, // pitch
            90 - 26.36 // yaw
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

        Logger.recordOutput("Pose/EstimatedPose", estimator.getEstimatedPosition());
    }

    private void driveToPosition(Pose2d currentPosition, Pose2d targetPosition) {
        Logger.recordOutput("targetPosition", targetPosition);
        if (currentPosition.getX() - targetPosition.getX() > 3) {
            return;
        }

        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12); // TODO: lower if needed
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

        List<Waypoint> waypoints = List.of(
            new Waypoint(currentPosition.getTranslation(), currentPosition.getTranslation(), currentPosition.getTranslation()),
            new Waypoint(targetPosition.getTranslation(), targetPosition.getTranslation(), targetPosition.getTranslation())
        );

        PathPlannerPath path = new PathPlannerPath(
                waypoints, // waypoints, add more if needed
                constraints,
                null, // starting, not needed
                new GoalEndState(0.0, targetPosition.getRotation())
        );

        path.preventFlipping = true;

        Command followPath = AutoBuilder.followPath(path);

        Command cleanupCommand = Commands.run(() -> {
           this.isRunningPath = false;
           this.runningCommand = null;
        });

        followPath = followPath.andThen(cleanupCommand);

        followPath.schedule();

        runningCommand = followPath;

        isRunningPath = true;
    }

    public void interruptPath() {
        if (runningCommand != null) {
            runningCommand.cancel();
            runningCommand = null;
        }
        isRunningPath = false;
    }

    public void runPath(AprilTagLineups lineup, Pose2d currentPose) {
        if (isRunningPath) {
            return;
        }

        driveToPosition(currentPose, lineup.getPose());
    }

    public void runPath(AprilTagLineups lineup, CoralLineup positionTranslation, Pose2d currentPose) {
        if (isRunningPath) {
            return;
        }

        Pose2d position = lineup.getPose();
        position = new Pose2d(position.getX() + positionTranslation.getXTranslation(), position.getY(), position.getRotation());

        driveToPosition(position, currentPose);
    }

    public AprilTagLineups getClosestTag(Pose2d currentPose) {
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

    @Override
    public void periodic() {
        if (runningCommand != null) {
            Logger.recordOutput("isFinished", !runningCommand.isFinished());
        } else {
            Logger.recordOutput("isFinished", false);
        }
        Logger.recordOutput("isRunningPath", isRunningPath);
    }
}
