package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TargetPositions {
    private static final double ELEVATOR_OFFSET = -0.0508; // meters distance from center of robot

    private static final double REEF_COLUMN_OFFSET = 0.1778; // meters distance from center of a reef side

    private static Translation2d REEF_CENTER = new Translation2d(4.5, 4.0);

    private static Pose2d REEF_FAR = new Pose2d(5.75, 4.00, Rotation2d.fromDegrees(0));
    private static Pose2d REEF_FAR_RIGHT = new Pose2d(5.10, 2.93, Rotation2d.fromDegrees(-60));
    private static Pose2d REEF_NEAR_RIGHT = new Pose2d(3.87, 2.93, Rotation2d.fromDegrees(-120));
    private static Pose2d REEF_NEAR = new Pose2d(3.23, 4.00, Rotation2d.fromDegrees(180));
    private static Pose2d REEF_NEAR_LEFT = new Pose2d(3.87, 5.13, Rotation2d.fromDegrees(120));
    private static Pose2d REEF_FAR_LEFT = new Pose2d(5.10, 5.13, Rotation2d.fromDegrees(60));

    public TargetPositions() {
        Logger.recordOutput("Targets/ReefFar", REEF_FAR);
        Logger.recordOutput("Targets/ReefFarRight", REEF_FAR_RIGHT);
        Logger.recordOutput("Targets/ReefFarLeft", REEF_FAR_LEFT);
        Logger.recordOutput("Targets/ReefNearRight", REEF_NEAR_RIGHT);
        Logger.recordOutput("Targets/ReefNearLeft", REEF_NEAR_LEFT);
        Logger.recordOutput("Targets/ReefNear", REEF_NEAR);
        Logger.recordOutput("Targets/ReefCenter", REEF_CENTER);
    }

    // get robot position on field. Use position to get the section of the field
    

    // get the side of the reef that the robot is on by getting angle between center of robot and center of reef (REEF_CENTER)


    // get a target position for the robot by checking whick area of the field the robot is in and the side of the field that it is on
}
