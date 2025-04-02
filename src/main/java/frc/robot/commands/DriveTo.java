package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

import static edu.wpi.first.units.Units.Meters;

public class DriveTo extends Command {
    //Variable for local instance of Swerve subsystem
    private Drive m_swerve;
    //Variable for local copy of the pose for the target to pathfind to
    private Pose2d m_target;
    //Variable for local copy of the PID config
    private PPHolonomicDriveController m_driveController;
    //Variable for local copy of Pathplanner state
    private PathPlannerTrajectoryState m_state;
    //Variable for local copy of the maximum faliure treshold
    private static final double kTreshM = 0.01;

    /**
     *  Aligns to a certain target
     *
     * @param swerve the swerve subsystem
     * @param target the Pose of where to pathfind to
     */
    public DriveTo(Drive swerve, Pose2d target) {
        m_swerve = swerve;
        m_target = target;

        m_driveController =
                new PPHolonomicDriveController(
                        new PIDConstants(5, 0, 0),
                        new PIDConstants(4, 0, 0),
                        0.02); // loop time
        m_state = new PathPlannerTrajectoryState();

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        //sets the pathplanner ending state to our target
        m_state.pose = m_target;
    }

    @Override
    public void execute() {
        //Calculate the speed and heading of each swerve module
        ChassisSpeeds speeds = m_driveController.calculateRobotRelativeSpeeds(m_swerve.getPose(), m_state);
        //Drive with those speeds and headings
        m_swerve.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        //Stop the swerve modules when the command ends
        m_swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.hypot(
                getXDistance(m_swerve.getPose(), m_target),
                getYDistance(m_swerve.getPose(), m_target)
        ) <= kTreshM &&
                dot(m_swerve.getRotation(), m_target.getRotation()) >= Math.cos(Units.degreesToRadians(0.5));
    }

    public static double getXDistance(Pose2d a, Pose2d b) {
        return Math.abs(a.getMeasureX().in(Meters) - b.getMeasureX().in(Meters));
    }

    public static double getYDistance(Pose2d a, Pose2d b) {
        return Math.abs(a.getMeasureY().in(Meters) - b.getMeasureY().in(Meters));
    }

    public static double dot(Rotation2d a, Rotation2d b) {
        return a.getCos() * b.getCos() + a.getSin() * b.getSin(); // 2d dot product: a_x * b_x + a_y * b_y
    }
}