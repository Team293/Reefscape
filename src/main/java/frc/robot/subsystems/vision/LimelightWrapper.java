package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

public class LimelightWrapper extends SubsystemBase implements TagCamera {
    private final String m_name;
    private final Supplier<Rotation3d> m_imuRotation;
    private final Supplier<Transform3d> m_camTransform;
    private LimelightHelpers.PoseEstimate m_poseEstimateMT1;
    private LimelightHelpers.PoseEstimate m_poseEstimateMT2;
    private final static Matrix<N3, N1> kDefaultStdv = VecBuilder.fill(2, 2, 3); //TODO: adjust this
    private final static Matrix<N3, N1> kDefaultStdvMT2 = VecBuilder.fill(2, 2, Double.POSITIVE_INFINITY);
    private final Alert m_camDisconnected;
    private final DoubleSubscriber m_latencySubscriber;

    private VisionData m_dataReturned = new VisionData();

    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final boolean kExtraVisionDebugInfo = true;

    public LimelightWrapper(String name, Supplier<Transform3d> cameraTransform, Supplier<Rotation3d> imuRotation) {
        m_name = name.toLowerCase(); //hostname must be lowercase
        m_imuRotation = imuRotation;
        m_camTransform = cameraTransform;
        m_poseEstimateMT1 = null;
        m_poseEstimateMT2 = null;

        m_latencySubscriber = LimelightHelpers.getLimelightNTTable(m_name)
                .getDoubleTopic("tl").subscribe(0.0);
        m_camDisconnected = new Alert("Limelight " + m_name + " Disconnected!", Alert.AlertType.kError);
    }

    @Override
    public String getName() {
        return m_name;
    }

    private boolean hasPose(LimelightHelpers.PoseEstimate estimate) {
        return LimelightHelpers.validPoseEstimate(estimate);
    }

    private Matrix<N3, N1> getEstStdv(LimelightHelpers.PoseEstimate estimate) {
        if(!estimate.isMegaTag2)
            return kDefaultStdv.div(getTagAreas(estimate));
        else
            return kDefaultStdvMT2.div(getTagAreas(estimate));
    }

    private double getTagAreas(LimelightHelpers.PoseEstimate estimate) {
        if(!hasPose(estimate)) return 0;
        return estimate.avgTagArea * estimate.tagCount;
    }

    private LimelightHelpers.RawFiducial[] getTargets() {
        if(hasPose(m_poseEstimateMT1))
        {
            return m_poseEstimateMT1.rawFiducials;
        }
        return null;
    }

    public boolean checkVisionResult(LimelightHelpers.PoseEstimate estimate) {
        if(!hasPose(estimate)) return false;

        // fixme: area units are a bit different, so disable this check
        // if(getTagAreas() < 0.3) return false;

//        if(Math.abs(estimate.pose.getZ()) > 0.5) return false; fixme

        //the constrained PNP should have no ambiguity
        if(!estimate.isMegaTag2)
        {
            if(getTargets().length == 1) {
                if(getTargets()[0].ambiguity > 0.6) {
                    return false;
                }
            }
        }

        return true;
    }

    private boolean isConnected() {
        long dt_us = RobotController.getFPGATime() - m_latencySubscriber.getLastChange();
        double dt = Seconds.convertFrom(dt_us, Microseconds);
        return dt < 0.25;
    }

    private void copyEstimateData(LimelightHelpers.PoseEstimate estimate) {
        m_dataReturned.pose = estimate.pose;
        m_dataReturned.timestamp = estimate.timestampSeconds;
        m_dataReturned.stdv = getEstStdv(estimate);
    }

    public void refreshEstimate(Consumer<VisionData> data) {
        /*
         * MT2 estimates have high rotation stdv so that MT1 can correct it.
         * MT1 estimates have a higher position stdv
         * since MT2 is more accurate once rotation is correct!
         */
        if (checkVisionResult(m_poseEstimateMT1)) {
            copyEstimateData(m_poseEstimateMT1);
            data.accept(m_dataReturned);
        }
        if (checkVisionResult(m_poseEstimateMT2)) {
            copyEstimateData(m_poseEstimateMT2);
            data.accept(m_dataReturned);
        }
    }

    private final ArrayList<Pose3d> targets = new ArrayList<>();

    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(m_name, m_imuRotation.get());
        LimelightHelpers.setCameraPose_RobotSpace(m_name, m_camTransform.get());
        m_poseEstimateMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name);
        m_poseEstimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name);

        m_camDisconnected.set(!isConnected());

        Logger.recordOutput(m_name + "/hasPose", hasPose(m_poseEstimateMT1) && hasPose(m_poseEstimateMT2));
        targets.clear();

        if(hasPose(m_poseEstimateMT1) && hasPose(m_poseEstimateMT2)) {
            Logger.recordOutput(m_name + "/pose3dMT1", m_poseEstimateMT1.pose);
            Logger.recordOutput(m_name + "/pose3dMT2", m_poseEstimateMT2.pose);
            Logger.recordOutput(m_name + "/cameraTransform", m_camTransform.get());

            /* should be same for both from here and below */
            Logger.recordOutput(m_name + "/tagArea", getTagAreas(m_poseEstimateMT1));

            /* if the pose estimate is valid then getTargets() != null */
            LimelightHelpers.RawFiducial[] fiducials = getTargets();
            Logger.recordOutput(m_name + "/ambiguity", fiducials.length == 1 ? fiducials[0].ambiguity : 0);

            if(kExtraVisionDebugInfo)
            {
                for (LimelightHelpers.RawFiducial f : fiducials) {
                    Optional<Pose3d> pose = kFieldLayout.getTagPose(f.id);
                    pose.ifPresent(targets::add);
                }
                Logger.recordOutput(m_name + "/visionTargets", targets.toArray(new Pose3d[targets.size()]));
            }
        }
    }
}
