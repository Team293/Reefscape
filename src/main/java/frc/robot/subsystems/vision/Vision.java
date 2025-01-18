package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;

public class Vision extends SubsystemBase {
    private final String NAME = "limelight";
    private final Drive drive;

    public Vision(Drive drive) {
        this.drive = drive;

        LimelightHelpers.setCameraPose_RobotSpace(NAME,
            0.2, // forward
            0.0, // side
            0.2, // up
            0.0, // roll
            0.0, // pitch
            0.0 // yaw
        );
    }

    public void init(SwerveDrivePoseEstimator estimator) {
        estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    }

    public void updatePose(SwerveDrivePoseEstimator estimator, Pose2d robotPose) {
        int tags = LimelightHelpers.getRawFiducials("limelight").length;

        if (tags > 0) {
            estimator.addVisionMeasurement(getVisionPose(), Timer.getFPGATimestamp());
        }
    }

    public Pose2d getVisionPose() {
        return LimelightHelpers.getBotPose2d_wpiBlue("limelight");
    }

    public double distanceToFirstTarget(Pose2d robotPose) {
        return getVisionPose().getTranslation().getDistance(robotPose.getTranslation());
    }
}
