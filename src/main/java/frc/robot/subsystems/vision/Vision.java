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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
import frc.robot.util.LimelightHelpers;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class Vision extends SubsystemBase {
    private static final String[] LIMELIGHT_NAMES = {
        "limelight-left",
        "limelight-right",
        "limelight-top"
    };

    private static final double MAX_DISTANCE = 0.5; // in

    private boolean isRunningPath = false;
    private Command runningCommand;
    private Pose2d targetPose;
    private Supplier<Pose2d> currentPose;
    
    private final XboxController driveController;
    private final XboxController operatorController;

    private Pose2d[] poses = new Pose2d[LIMELIGHT_NAMES.length];
    private boolean[] updated = new boolean[LIMELIGHT_NAMES.length];

    public enum AprilTagLineups {
        CORAL_1(new Pose2d(1.21, 6.95, Rotation2d.fromDegrees(-53 + 180))),
        CORAL_2(new Pose2d(1.06, 1.11, Rotation2d.fromDegrees(55 + 180))),
        RED_6(new Pose2d(3.87, 5.13, Rotation2d.fromDegrees(120 + 180))),
        RED_7(new Pose2d(3.23, 4.00, Rotation2d.fromDegrees(180 + 180))),
        RED_8(new Pose2d(3.87, 2.93, Rotation2d.fromDegrees(-120 + 180))),
        RED_9(new Pose2d(5.10, 2.93, Rotation2d.fromDegrees(-60 + 180))),
        RED_10(new Pose2d(5.75, 4.00, Rotation2d.fromDegrees(180))), // reef far
        RED_11(new Pose2d(5.10, 5.13, Rotation2d.fromDegrees(60 + 180))),
        CORAL_12(new Pose2d(1.06, 1.11, Rotation2d.fromDegrees(55 + 180))),
        CORAL_13(new Pose2d(1.21, 6.95, Rotation2d.fromDegrees(-53 + 180))),
        BLUE_16(new Pose2d(3.87, 5.13, Rotation2d.fromDegrees(120 + 180))),
        BLUE_18(new Pose2d(3.23, 4.00, Rotation2d.fromDegrees(180 + 180))),
        BLUE_17(new Pose2d(3.87, 2.93, Rotation2d.fromDegrees(-120 + 180))),
        BLUE_19(new Pose2d(5.10, 2.93, Rotation2d.fromDegrees(-60 + 180))),
        BLUE_20(new Pose2d(5.10, 5.13, Rotation2d.fromDegrees(60 + 180))),
        BLUE_21(new Pose2d(5.75, 4.00, Rotation2d.fromDegrees(180 + 180)));
        private final Pose2d pose;

        AprilTagLineups(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

    public enum CoralLineup {
        LEFT(0.22d),
        RIGHT(-0.10d);

        private final double xTranslation;

        CoralLineup(double xTranslation) {
            this.xTranslation = xTranslation;
        }

        public double getXTranslation() {
            return xTranslation;
        }
    }

    private static final Rotation2d[] LIMELIGHT_YAW_OFFSETS = {
        Rotation2d.fromDegrees(-22.48),
        Rotation2d.fromDegrees(26.36),
        Rotation2d.fromDegrees(0)
    };

    public void setPositionSupplier(Supplier<Pose2d> position) {
        this.currentPose = position;
    }

    public Vision(XboxController driveController, XboxController operatorController) {
        LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_NAMES[0],
            0.24765, // forward
            -0.2159, // side
            0.3302, // up
            180, // roll
            0.0, // pitch
                -26.5 // yaw
        );

        LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_NAMES[1],
            0.2477516, // forward
            0.25495, // side
            0.3302, // up
            180, // roll
            0, // pitch
            26.5 // yaw
        );

        LimelightHelpers.setCameraPose_RobotSpace(LIMELIGHT_NAMES[2],
            -0.05, // forward
            -0.2159, // side
            1.0, // up
            90, // roll
            24, // pitch
            -2 // yaw
        );

        // force limelights to use roboRIO for gyro
        for (String name : LIMELIGHT_NAMES) {
            LimelightHelpers.SetIMUMode(name, 0);
        }

        for (AprilTagLineups lineup : AprilTagLineups.values()) {
            Logger.recordOutput("TargetsVision/" + lineup.name(), lineup.getPose());
        }

        this.driveController = driveController;
        this.operatorController = operatorController;
    }

    public void updateRobotPose(SwerveDrivePoseEstimator estimator, double gyroRate, GyroIOInputsAutoLogged gyroInputs, Drive drive) {
        if (Math.abs(gyroRate) > 720) {
            return;
        }
        
        int visibleTags = 0;
        int i = 0;
        for (String name : LIMELIGHT_NAMES) {
            if(setVisionPoseEstimation(i, name, estimator, gyroInputs, LIMELIGHT_YAW_OFFSETS[i])) {
                visibleTags++;
            }
            i++;
        }
        
        ArrayList<Double> degrees = new ArrayList<Double>();

        i = 0;
        for (Pose2d pose : poses) {
            if (updated[i]) {
                degrees.add(pose.getRotation().getDegrees());
            }
            i++;
        }
        
        Logger.recordOutput("VisibleTags", visibleTags);
        if (visibleTags >= 2) {
            double avgRot = averageAngles(degrees);
            Logger.recordOutput("AverageRotation", avgRot);
            drive.resetRotation(avgRot);
        }
    }

    public static double averageAngles(ArrayList<Double> angles) {
        double sumX = 0.0;
        double sumY = 0.0;

        for (double angle : angles) {
            sumX += Math.cos(Math.toRadians(angle));
            sumY += Math.sin(Math.toRadians(angle));
        }

        double avgAngle = Math.toDegrees(Math.atan2(sumY, sumX));

        return avgAngle;
    }

    private boolean setVisionPoseEstimation(int id, String limelightName, SwerveDrivePoseEstimator estimator, GyroIOInputs gyroInputs, Rotation2d yawOffset) {
        boolean rejectOdometry = false;
        updated[id] = false;

        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        }

        if (poseEstimate == null) return false;

        if (poseEstimate.rawFiducials.length < 1) {
            return false;
        }

        if (poseEstimate.rawFiducials[0].ambiguity > 0.7) {
            return false;
        }

        if (poseEstimate.rawFiducials[0].distToCamera > 1.5) {
            return false;
        }

        Pose2d visionPose;

        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            rejectOdometry = true;
            return false;
        }

        if (!rejectOdometry) {
            visionPose = poseEstimate.pose;
            
            Logger.recordOutput("Limelight/EstimatedPose-" + limelightName, visionPose);
            
            estimator.addVisionMeasurement(visionPose, poseEstimate.timestampSeconds);
            poses[id] = visionPose;
            updated[id] = true;
        } else {
            return false;
        }

        Logger.recordOutput("Pose/EstimatedPose", estimator.getEstimatedPosition());
        return true;
    }

    private void driveToPosition(Pose2d currentPosition, Pose2d targetPosition) {
        Logger.recordOutput("targetPosition", targetPosition);
        System.out.println("driveToPosition");

        if (isRunningPath) {
            return;
        }
        
        this.targetPose = targetPosition;

        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12); // TODO: lower if needed
        PathConstraints constraints = new PathConstraints(1.5, 1.5, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

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
           this.targetPose = null;
        });

        followPath = followPath.andThen(cleanupCommand);

        followPath.schedule();

        runningCommand = followPath;

        isRunningPath = true;
    }

    public void interruptPath() {
        if (runningCommand != null && isRunningPath) {
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

    public void runPath(CoralLineup positionTranslation, Pose2d currentPose) {
        if (isRunningPath) {
            return;
        }

        Pose2d position = targetPose;


        driveToPosition(currentPose, position);
    }

    public Pose2d applyTranslation(CoralLineup side, Pose2d poseToTranslate) {
        if (poseToTranslate == null) return new Pose2d();

        double translateX = side.xTranslation * Math.sin(-poseToTranslate.getRotation().getRadians());
        double translateY = side.xTranslation * Math.cos(-poseToTranslate.getRotation().getRadians());

        return new Pose2d(poseToTranslate.getX() + translateX, poseToTranslate.getY() + translateY, poseToTranslate.getRotation());
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

    public boolean isRunningPath() {
        return isRunningPath;
    }

    @Override
    public void periodic() {
        if (runningCommand != null) {
            Logger.recordOutput("isFinished", runningCommand.isFinished());
        } else {
            Logger.recordOutput("isFinished", false);
        }
        
        if (isRunningPath && targetPose != null) {
            if (isFinished()) {
                interruptPath();
            }
        }

        Pose2d robotPose = currentPose.get();
        Pose2d closestTagPose = getClosestTag(robotPose).pose;

        Pose2d closestLeft = applyTranslation(CoralLineup.LEFT, closestTagPose);
        Pose2d closestRight = applyTranslation(CoralLineup.RIGHT, closestTagPose);

        if (isCloseToPose(robotPose, closestLeft) || isCloseToPose(robotPose, closestRight)) {
            // close to either poses
            this.driveController.setRumble(RumbleType.kBothRumble, 0.2);
            this.operatorController.setRumble(RumbleType.kBothRumble, 0.2);
        } else {
            this.driveController.setRumble(RumbleType.kBothRumble, 0);
            this.operatorController.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    public boolean isFinished() {
        Pose2d currentPose = this.currentPose.get();

        return isCloseToPose(currentPose, targetPose);
    }

    public boolean isCloseToPose(Pose2d currentPose, Pose2d poseToCheck) {
        double distanceX = Math.abs(poseToCheck.getX() - currentPose.getX()) * 39.37;
        double distanceY = Math.abs(poseToCheck.getY() - currentPose.getY()) * 39.37;

        boolean closeToDeadband = distanceX <= MAX_DISTANCE;
        boolean closeToNegativeDeadband = distanceY <= MAX_DISTANCE;

        return closeToDeadband || closeToNegativeDeadband;
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    public Pose2d closestTargetPose(Pose2d currentPose, CoralLineup side) {
        return applyTranslation(side, getClosestTag(currentPose).pose);
    }
}