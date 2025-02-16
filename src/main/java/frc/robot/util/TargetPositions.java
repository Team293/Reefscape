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

    //Get robot position on field. Use position to get the section of the field
    public Pose2d getPositionClosestToRobotPosition()
    {
        Pose2d robotPosition = new Pose2d(); //Put in actual position
        Pose2d[] targetPositions = {REEF_FAR, REEF_FAR_LEFT, REEF_FAR_RIGHT, REEF_NEAR, REEF_NEAR_LEFT, REEF_NEAR_RIGHT};
        //Sets the closest position to REEF_FAR
        Pose2d closestPosition = targetPositions[0]; 
        //Gets the distance between the robot position and closest target position
        double minDistance = getDistance(robotPosition, closestPosition); 
        for (Pose2d target : targetPositions) {
            //Gets the distance between the robot and the target positions
            double distance = getDistance(robotPosition, target); 
            //If the distance is shorter than the minimum distance set the target position to the closest target position
            if (distance < minDistance) { 
                minDistance = distance;
                closestPosition = target;
            }
        }
        return closestPosition;
    }

    public double getDistance(Pose2d robotPosition, Pose2d target) {
        double dx = robotPosition.getX() - target.getX();
        double dy = robotPosition.getY() - target.getY();
        //returns the distance between the center of the robot and the center of the target position
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2)); 
    }
        
    // get the side of the reef that the robot is on by getting angle between center of robot and center of reef (REEF_CENTER)
    public double getAngle(Pose2d robotPosition, Pose2d target) {
        // Record how off the robot is from the center of reef on the y-axis and x-axis
        double dx = robotPosition.getX() - target.getX();
        double dy = robotPosition.getY() - target.getY();
        // Take the inverse tan of the two values to get the angle
        return Math.toDegrees(Math.atan2(dy, dx));
    }
    // get a target position for the robot by checking whick area of the field the robot is in and the side of the field that it is on
}
