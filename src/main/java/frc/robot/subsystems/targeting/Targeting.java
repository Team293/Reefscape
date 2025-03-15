package frc.robot.subsystems.targeting;

import static edu.wpi.first.units.Units.Ohm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class Targeting extends SubsystemBase {
    private static final double ELEVATOR_OFFSET = -0.0508; // meters distance from center of robot

    private static final double REEF_COLUMN_OFFSET = 0.1778; // meters distance from center of a reef side

    private static Translation2d REEF_CENTER = new Translation2d(4.5, 4.0);

    private static Pose2d REEF_FAR = new Pose2d(5.75, 4.00, Rotation2d.fromDegrees(0 + 180));
    private static Pose2d REEF_FAR_RIGHT = new Pose2d(5.10, 2.93, Rotation2d.fromDegrees(-60 + 180));
    private static Pose2d REEF_NEAR_RIGHT = new Pose2d(3.87, 2.93, Rotation2d.fromDegrees(-120 + 180));
    private static Pose2d REEF_NEAR = new Pose2d(3.23, 4.00, Rotation2d.fromDegrees(180 + 180));
    private static Pose2d REEF_NEAR_LEFT = new Pose2d(3.87, 5.13, Rotation2d.fromDegrees(120 + 180));
    private static Pose2d REEF_FAR_LEFT = new Pose2d(5.10, 5.13, Rotation2d.fromDegrees(60 + 180));
    private static Pose2d RIGHT_CORAL_STATION = new Pose2d(1.06, 1.11, Rotation2d.fromDegrees(55 + 180));
    private static Pose2d ALGAE_SCORE = new Pose2d(5.8,0.77,Rotation2d.fromDegrees(-90 + 180));
    private static Pose2d LEFT_CORAL_STATION = new Pose2d(1.21, 6.95, Rotation2d.fromDegrees(-53 + 180));
    private final Drive drive;

    public Targeting(Drive drive) {
        this.drive = drive;

        Logger.recordOutput("Targets/ReefFar", REEF_FAR);
        Logger.recordOutput("Targets/ReefFarRight", REEF_FAR_RIGHT);
        Logger.recordOutput("Targets/ReefFarLeft", REEF_FAR_LEFT);
        Logger.recordOutput("Targets/ReefNearRight", REEF_NEAR_RIGHT);
        Logger.recordOutput("Targets/ReefNearLeft", REEF_NEAR_LEFT);
        Logger.recordOutput("Targets/ReefNear", REEF_NEAR);
        Logger.recordOutput("Targets/ReefCenter", REEF_CENTER);
        Logger.recordOutput("Targets/RightCoralStation", RIGHT_CORAL_STATION);
        Logger.recordOutput("Targets/AlgaeScore", ALGAE_SCORE);
        Logger.recordOutput("Targets/LeftCoralStation", LEFT_CORAL_STATION);
    }

    //Get robot position on field. Use position to get the section of the field
    @AutoLogOutput(key = "Targets/ClosestTarget")
    public Pose2d getPositionClosestToRobotPosition()
    {
        Pose2d robotPosition = drive.getPose();
        Pose2d[] targetPositions = {REEF_FAR, REEF_FAR_LEFT, REEF_FAR_RIGHT, REEF_NEAR, REEF_NEAR_LEFT, REEF_NEAR_RIGHT, RIGHT_CORAL_STATION, ALGAE_SCORE, LEFT_CORAL_STATION};
        Pose2d closestPosition = targetPositions[0]; 
        double minDistance = getDistance(robotPosition, closestPosition); 
        for (Pose2d target : targetPositions) {
            double distance = getDistance(robotPosition, target); 
            if (distance < minDistance) { 
                minDistance = distance;
                closestPosition = target;
            }
        }
        return closestPosition;
    }

    public double getDistance(Pose2d robotPosition, Pose2d target) {
        return robotPosition.getTranslation().getDistance(target.getTranslation()); 
    }
        
    // get the side of the reef that the robot is on by getting angle between center of robot and center of reef (REEF_CENTER)
    public double getAngle(Pose2d robotPosition) {
        // Record how off the robot is from the center of reef on the y-axis and x-axis
        double dx = robotPosition.getX() - REEF_CENTER.getX();
        double dy = robotPosition.getY() - REEF_CENTER.getY();
        // Take the inverse tan of the two values to get the angle
        return Math.toDegrees(Math.atan2(dy, dx));
    }
}
