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
import frc.robot.commands.ReverseAlgaeKnocker;
import frc.robot.commands.DropCoral;
import frc.robot.commands.EnableAlgaeKnocker;
import frc.robot.commands.EnableAlgaePickup;
import frc.robot.commands.PickupCoral;
import frc.robot.commands.ReverseAlgaePickup;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.commands.SubsystemControl;
import frc.robot.subsystems.algaePickup.AlgaePickup;
import frc.robot.subsystems.algaeknocker.AlgaeKnocker;
import frc.robot.subsystems.coralScorer.CoralScorer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
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
  // private final CoralScorer coralScorer;
  // private final AlgaePickup algaePickup;
  private final Vision vision;
  // private final AlgaeKnocker algaeKnocker;
  // private final Elevator elevator;

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

    // algaePickup = new AlgaePickup();
    // elevator = new Elevator();
    // coralScorer = new CoralScorer();
    vision = new Vision();
    // algaeKnocker = new AlgaeKnocker();

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

    // NamedCommands.registerCommand("pickupCoral", new PickupCoral(coralScorer));
    // NamedCommands.registerCommand("dropCoral", new DropCoral(coralScorer));
    // NamedCommands.registerCommand("dropCoral2", new DropCoral(coralScorer));
    //NamedCommands.registerCommand("elevatorToL2", new SetElevatorHeight(elevator, 2, 20));
    // NamedCommands.registerCommand("enableAlgaePickup", new EnableAlgaePickup(algaePickup));
    // NamedCommands.registerCommand("reverseAlgaePickup", new ReverseAlgaePickup(algaePickup));
    // NamedCommands.registerCommand("enableAlgaeKnocker", new EnableAlgaeKnocker(algaeKnocker));
    // NamedCommands.registerCommand("disableAlgaeKnocker", new ReverseAlgaeKnocker(algaeKnocker));
    //NamedCommands.registerCommand("elevatorToL4", new SetElevatorHeight(elevator, 4, 20)); //Check if correct

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
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> driverController.getLeftTriggerAxis(),
            () -> driverController.getRightTriggerAxis()));
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

    /* Intake auto-run command */
    /* Reverse intake control as well */
    // algaePickup.setDefaultCommand(
    //     SubsystemControl.algaePickup(
    //         algaePickup,
    //         operatorController::getRightY));    
    
    driverController
        .a()
        .onTrue(Commands.runOnce(() -> drive.resetRotation(180.0), drive).ignoringDisable(true));

    // elevator.setDefaultCommand(SubsystemControl.elevatorControl(
    //   elevator,
    //   // operatorController::getLeftY,
    //   () -> operatorController.rightStick().getAsBoolean(),
    //   operatorController
    // ));

    // coralScorer.setDefaultCommand(
    //   SubsystemControl.coralControl(
    //     coralScorer, 
    //     operatorController::getLeftY, 
    //     () -> operatorController.leftStick().getAsBoolean()
    //   )
    // );

    // operatorController.leftBumper().onTrue(Commands.runOnce(() -> algaeKnocker.enableAlgaeKnocker(), algaeKnocker));
    // operatorController.rightBumper().onTrue(Commands.runOnce(() -> algaeKnocker.disableAlgaeKnocker(), algaeKnocker));
  }
    //     .onTrue(Commands.runOnce(() -> drive.resetRotation(180.0), drive).ignoringDisable(true));
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
