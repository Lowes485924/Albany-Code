// Copyright 2021-2025 FRC 6328
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algae.algaeIntake;
import frc.robot.subsystems.algae.algaeIntakeWrist;
import frc.robot.subsystems.algae.flywheel;
import frc.robot.subsystems.algae.indexer;
import frc.robot.subsystems.coral.coralIntake;
import frc.robot.subsystems.coral.coralIntakeWrist;
import frc.robot.subsystems.coral.elevator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private algaeIntake algaeIntake;
  private algaeIntakeWrist algaeIntakeWrist;
  private indexer indexer;
  private coralIntake coralIntake;
  private flywheel flywheel;
  private coralIntakeWrist coralIntakeWrist;
  private elevator elevator;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController c2 = new CommandXboxController(1);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    /**
     * Gets the current position of the elevator.
     *
     * @return The elevator position as a double.
     */
    algaeIntake = new algaeIntake();
    indexer = new indexer();
    coralIntake = new coralIntake();
    flywheel = new flywheel();
    coralIntakeWrist = new coralIntakeWrist();
    elevator = new elevator();
    algaeIntakeWrist = new algaeIntakeWrist();

    NamedCommands.registerCommand("eL4", elevator.eL4Command());
    NamedCommands.registerCommand("eL1", elevator.eL1Command());
    NamedCommands.registerCommand("eSource", elevator.eSourceCommand());
    NamedCommands.registerCommand("wL4", coralIntakeWrist.ciwL4Command());
    NamedCommands.registerCommand("wSource", coralIntakeWrist.ciwSourceCommand());
    NamedCommands.registerCommand("score", coralIntake.coralScoreCommand());
    NamedCommands.registerCommand("load", coralIntake.coralLoadCommand());

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        Commands.run(
            () -> {
              if (elevator.getElevatorPosition().getValueAsDouble() > 40) {
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY() / 1.6,
                        () -> -controller.getLeftX() / 1.6,
                        () -> -controller.getRightX() / 1.4)
                    .execute();
              } else {
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX() / 1.4)
                    .execute();
              }
            },
            drive));

    // Reset gyro to 0° when 7 button is pressed
    controller
        .button(7)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    c2.povUp().onTrue(Commands.parallel(coralIntakeWrist.ciwDCommand(), elevator.hdCommand()));
    c2.povDown().onTrue(Commands.parallel(coralIntakeWrist.ciwDCommand(), elevator.ldCommand()));
    c2.button(8)
        .onTrue(Commands.parallel(elevator.eSourceCommand(), coralIntakeWrist.ciwSourceCommand()));
    c2.button(7)
        .onTrue(Commands.parallel(elevator.eStoreCommand(), coralIntakeWrist.ciwStoreCommand()));
    c2.a().onTrue(Commands.parallel(elevator.eL1Command(), coralIntakeWrist.ciwL1Command()));
    c2.x().onTrue(Commands.parallel(elevator.eL2Command(), coralIntakeWrist.ciwL23Command()));
    c2.b().onTrue(Commands.parallel(elevator.eL3Command(), coralIntakeWrist.ciwL4Command()));
    c2.y().onTrue(Commands.parallel(elevator.eL4Command(), coralIntakeWrist.ciwL4Command()));

    controller
        .button(8)
        .onTrue(Commands.parallel(elevator.eSourceCommand(), coralIntakeWrist.ciwSourceCommand()));

    controller
        .a()
        .onTrue(Commands.parallel(elevator.eL1Command(), coralIntakeWrist.ciwL1Command()));
    controller
        .x()
        .onTrue(Commands.parallel(elevator.eL2Command(), coralIntakeWrist.ciwL23Command()));
    controller
        .b()
        .onTrue(Commands.parallel(elevator.eL3Command(), coralIntakeWrist.ciwL4Command()));
    controller
        .y()
        .onTrue(Commands.parallel(elevator.eL4Command(), coralIntakeWrist.ciwL4Command()));
    controller.rightBumper().whileTrue(coralIntake.coralScoreCommand());
    controller.rightTrigger().whileTrue(coralIntake.coralDropCommand());
    controller
        .rightBumper()
        .or(controller.rightTrigger())
        .whileFalse(coralIntake.coralLoadCommand());

    controller
        .leftTrigger()
        .onTrue(new SequentialCommandGroup(indexer.iShootCommand(), flywheel.fIntakeCommand()));

    controller.leftBumper().whileTrue(indexer.iStopCommand());
    controller.leftBumper().or(controller.leftTrigger()).whileFalse(indexer.iIntakeCommand());

    controller.leftBumper().onTrue(algaeIntake.aIntakeCommand());
    controller.leftBumper().onFalse(algaeIntake.aStopCommand());
    controller.povLeft().onTrue(algaeIntake.aDropCommand());
    c2.leftBumper().onTrue(algaeIntake.aShootCommand());

    c2.leftBumper().onTrue(flywheel.fShootCommand());

    c2.leftTrigger().onTrue(flywheel.fStopCommand());
    c2.leftTrigger().onTrue(algaeIntake.aStopCommand());
    c2.leftTrigger().onTrue(indexer.iStopCommand());

    controller
        .leftBumper()
        .and(controller.rightBumper())
        .whileTrue(algaeIntakeWrist.awMidCommand());
    controller
        .leftBumper()
        .and(controller.rightBumper().negate())
        .whileTrue(algaeIntakeWrist.awOutCommand());
    controller.leftBumper().whileFalse(algaeIntakeWrist.awInCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
