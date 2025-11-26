// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RollerCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANRollerSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANRollerSubsystem rollerSubsystem = new CANRollerSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up command bindings
    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoChooser.setDefaultOption("Autonomous", Commands.none());
    autoChooser.addOption("platform_right", getRightAutonomousCommand());
    autoChooser.addOption("platform_left (curves)", getLeftAutonomousCommand());

    SmartDashboard.putData("auto chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Set the A button to run the "RollerCommand" command with a fixed
    // value ejecting the gamepiece while the button is held

    // before
    // operatorController.a()
    //     .whileTrue(new RollerCommand(() -> RollerConstants.ROLLER_EJECT_VALUE, () -> 0, rollerSubsystem));

    driverController.a()
            .whileTrue(rollerSubsystem.c_rollerIntakeCommand());
    driverController.x()
            .whileTrue(rollerSubsystem.c_rollerReverseIntakeCommand());
    driverController.y()
            .whileTrue(new RunCommand(() -> driveSubsystem.arcadeDrive(0.5, 0), driveSubsystem).withTimeout(1));

    driverController.rightBumper()
            .whileTrue(rollerSubsystem.c_rotateArmUpCommand());
    driverController.leftBumper()
            .whileTrue(rollerSubsystem.c_rotateArmDownCommand());

    // Set the default command for the drive subsystem to an instance of the
    // DriveCommand with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). Similarly for the X axis where we need to flip the value so the
    // joystick matches the WPILib convention of counter-clockwise positive
    driveSubsystem.setDefaultCommand(new DriveCommand(
        () -> -driverController.getLeftY() *
            (driverController.getHID().getRightBumperButton() ? 1 : 0.5),
        () -> -driverController.getRightX(),
        driveSubsystem));

    // Set the default command for the roller subsystem to an instance of
    // RollerCommand with the values provided by the triggers on the operator
    // controller
    // rollerSubsystem.setDefaultCommand(new RollerCommand(
    //     () -> operatorController.getRightTriggerAxis(),
    //     () -> operatorController.getLeftTriggerAxis(),
    //     rollerSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
    /*return new ParallelDeadlineGroup(new RunCommand(() -> driveSubsystem.tankDrive(0.668419340845, 0.875145867316), driveSubsystem).withTimeout(2.5))
                  //.andThen(new ParallelDeadlineGroup(rollerSubsystem.c_rollerReverseIntakeCommand().withTimeout(0.5)))
                  .andThen(new ParallelDeadlineGroup(new RunCommand(() -> driveSubsystem.arcadeDrive(-0.5, 0), driveSubsystem).withTimeout(0.583)))
                  .andThen(new RunCommand(() -> driveSubsystem.arcadeDrive(0, 0.5), driveSubsystem).withTimeout(0.77))
                  .andThen(new RunCommand(() -> driveSubsystem.tankDrive(0.838754514641, 0.910853740518)).withTimeout(5));
                  //.andThen(rollerSubsystem.c_rollerIntakeCommand().withTimeout(0.5));*/
    // 90 degree turn at 0.5 speed = 0.77 seconds
    // 34 inches/second at 0.5 speed

    // START ON OTHER SIDE OF THE PLATFORM:

    /*return new RunCommand(() -> driveSubsystem.arcadeDrive(0.5, 0), driveSubsystem).withTimeout(2.47794117647)
                  .andThen(new RunCommand(() -> driveSubsystem.arcadeDrive(0, 0.5), driveSubsystem).withTimeout(0.77))
                  .andThen(new ParallelDeadlineGroup(new RunCommand(()-> driveSubsystem.arcadeDrive(0.5, 0), driveSubsystem).withTimeout(2.44117647059), rollerSubsystem.c_rotateArmUpCommand()))
                  .andThen(new ParallelDeadlineGroup(rollerSubsystem.c_rollerReverseIntakeCommand().withTimeout(0.5), rollerSubsystem.c_rotateArmUpCommand()))
                  .andThen(new ParallelDeadlineGroup(new RunCommand(() -> driveSubsystem.arcadeDrive(-0.5, 0), driveSubsystem).withTimeout(0.583), rollerSubsystem.c_rotateArmDownCommand()))
                  .andThen(new RunCommand(() -> driveSubsystem.arcadeDrive(0, 0.5), driveSubsystem).withTimeout(0.77))
                  .andThen(new RunCommand(() -> driveSubsystem.tankDrive(0.838754514641, 0.910853740518)).withTimeout(5));
                  .andThen(rollerSubsystem.c_rollerIntakeCommand().withTimeout(0.5));*/



    // JUNK:
                  /*.andThen(new RunCommand(() -> driveSubsystem.arcadeDrive(0, 0.5)).withTimeout(0.5))
                  .andThen(new ParallelDeadlineGroup(new RunCommand(() -> driveSubsystem.arcadeDrive(0.5, 0), driveSubsystem).withTimeout(1.125), rollerSubsystem.c_rotateArmUpCommand()))
                  .andThen(new ParallelDeadlineGroup(rollerSubsystem.c_rollerReverseIntakeCommand().withTimeout(0.5), rollerSubsystem.c_rotateArmUpCommand()))
                  .andThen(new ParallelDeadlineGroup(new RunCommand(() -> driveSubsystem.arcadeDrive(0.5, 0), driveSubsystem).withTimeout(0.5), rollerSubsystem.c_rotateArmDownCommand()))
                  .andThen(new RunCommand(() -> driveSubsystem.arcadeDrive(0, 0.5), driveSubsystem).withTimeout(0.2));*/


                  /*.andThen(new ParallelDeadlineGroup(new RunCommand(() -> driveSubsystem.arcadeDrive(0.5, 0.4025), driveSubsystem).withTimeout(1.1516), rollerSubsystem.c_rotateArmUpCommand()));
                  .andThen(new RunCommand(() -> rollerSubsystem.c_rollerReverseIntakeCommand(), rollerSubsystem).withTimeout(0.2))
                  .andThen(new ParallelCommandGroup(new RunCommand(() -> driveSubsystem.arcadeDrive(0.5, 0), driveSubsystem), new RunCommand(() -> rollerSubsystem.c_rotateArmDownCommand(), rollerSubsystem)).withTimeout(0.53))
                  .andThen(new RunCommand(() -> driveSubsystem.arcadeDrive(0, 0.5), driveSubsystem).withTimeout(0.12))
                  .andThen(new RunCommand(() -> driveSubsystem.arcadeDrive(0.5, 0.288), driveSubsystem).withTimeout(3.1825))
                  .andThen(new RunCommand(() -> rollerSubsystem.c_rollerIntakeCommand(), rollerSubsystem))
                  .andThen(new ParallelCommandGroup(new RunCommand(() -> driveSubsystem.arcadeDrive(-0.5, -0.288), driveSubsystem), new RunCommand(() -> rollerSubsystem.c_rotateArmUpCommand(), rollerSubsystem)).withTimeout(3.1825))
                  .andThen(new RunCommand(() -> driveSubsystem.arcadeDrive(0, -0.5), driveSubsystem).withTimeout(0.12))
                  .andThen(new RunCommand(() -> rollerSubsystem.c_rollerReverseIntakeCommand(), rollerSubsystem).withTimeout(0.2));*/
           
    // idfk if i even programmed this correctly man
    // watch one comma be out of place and the robot explodes
  }

  public Command getLeftAutonomousCommand(){
    return new ParallelDeadlineGroup(new RunCommand(() -> driveSubsystem.tankDrive(0.668419340845, 0.875145867316), driveSubsystem).withTimeout(2.5), rollerSubsystem.c_rotateArmUpCommand())
                      .andThen(new ParallelDeadlineGroup(rollerSubsystem.c_rollerReverseIntakeCommand().withTimeout(0.5)))
                      .andThen(new ParallelDeadlineGroup(new RunCommand(() -> driveSubsystem.arcadeDrive(-0.5, 0), driveSubsystem).withTimeout(0.583), rollerSubsystem.c_rotateArmDownCommand()))
                      .andThen(new RunCommand(() -> driveSubsystem.arcadeDrive(0, 0.5), driveSubsystem).withTimeout(0.77))
                      .andThen(new ParallelDeadlineGroup(new RunCommand(() -> driveSubsystem.tankDrive(0.838754514641, 0.910853740518)).withTimeout(5), rollerSubsystem.c_rotateArmDownCommand()))
                      .andThen(rollerSubsystem.c_rollerIntakeCommand().withTimeout(0.5));
  }

  public Command getRightAutonomousCommand(){
    return new RunCommand(() -> driveSubsystem.arcadeDrive(0.5, 0), driveSubsystem).withTimeout(2.47794117647)
                      .andThen(new RunCommand(() -> driveSubsystem.arcadeDrive(0, 0.5), driveSubsystem).withTimeout(0.77))
                      .andThen(new ParallelDeadlineGroup(new RunCommand(()-> driveSubsystem.arcadeDrive(0.5, 0), driveSubsystem).withTimeout(2.44117647059), rollerSubsystem.c_rotateArmUpCommand()))
                      .andThen(new ParallelDeadlineGroup(rollerSubsystem.c_rollerReverseIntakeCommand().withTimeout(0.5)))
                      .andThen(new ParallelDeadlineGroup(new RunCommand(() -> driveSubsystem.arcadeDrive(-0.5, 0), driveSubsystem).withTimeout(0.583), rollerSubsystem.c_rotateArmDownCommand()))
                      .andThen(new RunCommand(() -> driveSubsystem.arcadeDrive(0, 0.5), driveSubsystem).withTimeout(0.77))
                      .andThen(new ParallelDeadlineGroup(new RunCommand(() -> driveSubsystem.tankDrive(0.838754514641, 0.910853740518)).withTimeout(5), rollerSubsystem.c_rotateArmDownCommand()))
                      .andThen(rollerSubsystem.c_rollerIntakeCommand().withTimeout(0.5));
  }
}