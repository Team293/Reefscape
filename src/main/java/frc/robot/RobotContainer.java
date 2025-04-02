// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.SpikeController;
import frc.robot.commands.*;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.coralScorer.CoralScorer;
import frc.robot.subsystems.coralScorer.CoralScorer.States;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.subsystems.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final CoralScorer coralScorer;
  // private final AlgaePickup algaePickup;
  private final Vision vision;
  // private final AlgaeKnocker algaeKnocker;
  private final Elevator elevator;
  private final Pneumatics pneumatics;
  private final Climber climber;

  // Controller
  private static final double DEADBAND = 0.05;
  private final SpikeController driverController = new SpikeController(0, DEADBAND);
  private final SpikeController operatorController = new SpikeController(1, DEADBAND);

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /* Print the log directory */
    String logDir = DataLogManager.getLogDir();
    System.out.print(logDir);

    pneumatics = new Pneumatics();
    elevator = new Elevator();
    vision = new Vision(this.driverController.getHID(), this.operatorController.getHID());

    // algaePickup = new AlgaePickup();
    coralScorer = new CoralScorer();
    // algaeKnocker = new AlgaeKnocker(pneumatics);
    climber = new Climber(pneumatics);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                vision,
                new GyroIONavX(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        break;

       default:
        // Replayed robot, disable IO implementations
         drive =
             new Drive(
                vision,
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
         break;
     }

    vision.setPositionSupplier(() -> drive.getPose());

    // NamedCommands.registerCommand("enableAlgaePickup", new EnableAlgaePickup(algaePickup));
    // NamedCommands.registerCommand("reverseAlgaePickup", new ReverseAlgaePickup(algaePickup));
    // NamedCommands.registerCommand("dropCoral", new DropCoral(coralScorer));
    // NamedCommands.registerCommand("dropCoral2", new DropCoral(coralScorer));
    NamedCommands.registerCommand("pickupCoral", new PickupCoral(coralScorer));
    NamedCommands.registerCommand("pickupCoral2", new PickupCoral(coralScorer));
    
    NamedCommands.registerCommand("pointCoralDown", new SetCoralState(coralScorer, States.POINT_DOWN));
    NamedCommands.registerCommand("pointCoralDown2", new SetCoralState(coralScorer, States.POINT_DOWN));

    NamedCommands.registerCommand("dropCoral", new SetCoralState(coralScorer, States.DROP));
    NamedCommands.registerCommand("dropCoral2", new SetCoralState(coralScorer, States.DROP));
    
    NamedCommands.registerCommand("resetElevator", new ResetElevator(elevator));
    NamedCommands.registerCommand("resetElevator2", new ResetElevator(elevator));

    // NamedCommands.registerCommand("enableAlgaeKnocker", new EnableAlgaeKnocker(algaeKnocker));
    // NamedCommands.registerCommand("disableAlgaeKnocker", new ReverseAlgaeKnocker(algaeKnocker));
    
    NamedCommands.registerCommand("elevatorToL2", new SetElevator(elevator, 1));
    NamedCommands.registerCommand("elevatorToL3", new SetElevator(elevator, 2));
    NamedCommands.registerCommand("elevatorToL4", new SetElevator(elevator, 3));
    NamedCommands.registerCommand("elevatorToL42", new SetElevator(elevator, 3));
    NamedCommands.registerCommand("elevatorToCS", new SetElevator(elevator, 4));
    NamedCommands.registerCommand("elevatorToCS2", new SetElevator(elevator, 4));
    
    // Set up auto routines
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Set up feedforward characterization
    // autoChooser.addOption(
    //     "Drive FF Characterization",
    //     new FeedForwardCharacterization(
    //         drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Drive command */
    drive.setDefaultCommand(
        SubsystemControl.joystickDrive(
            drive,
            vision,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> driverController.getLeftTriggerAxis(), // TODO: change back to triggers
            () -> driverController.getRightTriggerAxis(),
            () -> driverController.leftBumper().getAsBoolean(),
            () -> driverController.rightBumper().getAsBoolean()));

    // vision.setDefaultCommand(
    //         SubsystemControl.visionDrive(
    //                 drive,
    //                 vision,
    //                 driverController
    //         )
    // );


    // driverController.leftBumper()
    // .onTrue(Commands.runOnce(() -> {
    //   Vision.CoralLineup offset = Vision.CoralLineup.LEFT;
    //   Vision.AprilTagLineups closestStation = vision.getClosestTag(drive.getPose());

    //   vision.setTargetPose(closestStation.getPose());

    //   if (vision.isFinished()) {
    //     return;
    //   }

    //   vision.runPath(offset, drive.getPose());
    // }, vision))
    // .onFalse(Commands.runOnce(vision::interruptPath, vision));

    // driverController.rightBumper()
    // .onTrue(Commands.runOnce(() -> {
    //   Vision.CoralLineup offset = Vision.CoralLineup.RIGHT;
    //   Vision.AprilTagLineups closestStation = vision.getClosestTag(drive.getPose());

    //   vision.setTargetPose(closestStation.getPose());

    //   if (vision.isFinished()) {
    //     return;
    //   }

    //   vision.runPath(offset, drive.getPose());
    // }, vision))
    // .onFalse(Commands.runOnce(vision::interruptPath, vision));

    /*
     * SubsystemControl.fieldOrientedRotation(
     * drive,
     * () -> -driverController.getLeftY(),
     * () -> -driverController.getLeftX(),
     * () -> {
     * Rotation2d rot =
     * new Rotation2d(driverController.getRightX(), driverController.getRightY());
     * double magnitude =
     * Math.hypot(driverController.getRightX(), driverController.getRightY());
     *
     * if (magnitude > 0.5) {
     * return (-rot.getDegrees() + 90) % 360;
     * } else {
     * return -1;
     * }
     * },
     * () -> driverController.getLeftTriggerAxis(),
     * () -> driverController.getRightTriggerAxis()));
     */

    /* Brake command */
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
   
    /* Reset heading command */
    driverController
        .y()
        .onTrue(Commands.runOnce(() -> drive.resetRotation(0.0), drive).ignoringDisable(true));
        
    /* Reset heading command */
    // driverController
    //     .a()
    //     .onTrue(Commands.runOnce(() -> drive.resetRotation(180.0), drive).ignoringDisable(true));

    //  algaePickup.setDefaultCommand(
    //     SubsystemControl.algaePickup(
    //          algaePickup,
    //          operatorController::getRightY,
    //          () -> operatorController.rightStick().getAsBoolean()));    
    
    driverController
        .a()
        .onTrue(Commands.runOnce(() -> drive.resetRotation(180.0), drive).ignoringDisable(true));

    driverController
    .b()
    .onTrue(Commands.runOnce(() -> vision.interruptPath(), vision));
     coralScorer.setDefaultCommand(
       SubsystemControl.coralControl(
         coralScorer, 
         operatorController::getLeftY,
         () -> operatorController.leftStick().getAsBoolean(),
         () -> operatorController.rightTrigger().getAsBoolean()
       )
     );

    elevator.setDefaultCommand(
      SubsystemControl.elevatorControl(elevator, operatorController)
    );

     climber.setDefaultCommand(
            SubsystemControl.climb(
                    climber,
                    () -> operatorController.getRightY() < -0.3,
                    () -> operatorController.getRightY() > 0.3,
                    operatorController.y()
            )
    );
    

    // algaeKnocker.setDefaultCommand(
    //    SubsystemControl.AlgaeKnocker(
    //      algaeKnocker, 
    //      () -> operatorController.rightBumper().getAsBoolean(), //L3
    //      () -> operatorController.leftBumper().getAsBoolean() //L2
    //    )
    //  );    operatorController.rightTrigger().onTrue(Commands.runOnce(() -> coralScorer.setState(States.INTAKE), coralScorer));
  }
    //     .onTrue(Commands.runOnce(() -> drive.resetRotation(180.0), drive).ignoringDisable(true));
  

  /** +
   * 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
