// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveArcadeTimerCommand;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.commands.DriveForwardTimerCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.subsystems.SimpleDriveTrainSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SimpleDriveTrainSubsystem m_simpleDriveTrainSubsystem = new SimpleDriveTrainSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandJoystick m_joystick = new CommandJoystick(0);


  // autonomous Command stuff
  private final SendableChooser<Command> m_chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /*
    m_simpleDriveTrainSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_simpleDriveTrainSubsystem,
                () -> -m_joystick.getRawAxis(OperatorConstants.kArcadeDriveSpeedAxis),
                () -> m_joystick.getRawAxis(OperatorConstants.kArcadeDriveTurnAxis))
    );
    */
    m_simpleDriveTrainSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_simpleDriveTrainSubsystem,
                () -> -accountForJitter("speed"),
                () -> -accountForJitter("turn"))
    );

    // autonomous Command stuff
    m_chooser = new SendableChooser<>();

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_simpleDriveTrainSubsystem::exampleCondition)
    //    .onTrue(new ArcadeDriveCommand(m_simpleDriveTrainSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // set Button.b as tankdrive for 50 revs at speed 0.4
    m_driverController.y().onTrue(new DriveForwardCommand(m_simpleDriveTrainSubsystem, SimulationConstants.simEncoderRevolutions, SimulationConstants.simSpeed));
    m_driverController.x().onTrue(new DriveForwardCommand(m_simpleDriveTrainSubsystem, 0., 0.));
  
    // set Button.a as tankdrive for time = 5 sec
    m_driverController.a().onTrue(new DriveForwardTimerCommand(m_simpleDriveTrainSubsystem, SimulationConstants.simTimer, SimulationConstants.simSpeed));
    
    // set Button.y as arcadedrive for time = 2.5 sec
    m_driverController.b().onTrue(new DriveArcadeTimerCommand(m_simpleDriveTrainSubsystem, SimulationConstants.simTimer / 2., SimulationConstants.simSpeed, SimulationConstants.simTurn));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    //
    // execute the timed autonoumous mode via a CommandGroup
    m_driverController.y().onTrue(new SequentialCommandGroup(
      new DriveForwardTimerCommand(m_simpleDriveTrainSubsystem, SimulationConstants.simTimerAutonoumous, SimulationConstants.simSpeed)));
    
    // try the same command with Auto.java interface
    m_driverController.x().onTrue(Autos.autoTimerDrive(m_simpleDriveTrainSubsystem));

    m_driverController.b().onTrue(new SequentialCommandGroup(
      new DriveForwardCommand(m_simpleDriveTrainSubsystem, 0., 0.)));
    
    //return new SequentialCommandGroup();
    //return null;
    // test on the robot - maybe the controller doesn't work on Autonomous??
    /**/
    return new SequentialCommandGroup(
      new DriveForwardTimerCommand(m_simpleDriveTrainSubsystem, SimulationConstants.simTimerAutonoumous, SimulationConstants.simSpeed));
    /**/
  }

  private double accountForJitter(String axisName) {
    double jitter = 0.;
    if (axisName == "speed") {
      jitter = m_joystick.getRawAxis(OperatorConstants.kArcadeDriveSpeedAxis);
    } else if (axisName == "turn") {
      jitter = m_joystick.getRawAxis(OperatorConstants.kArcadeDriveTurnAxis);
    }
    if ( Math.abs(jitter) < OperatorConstants.jitterTolerance) {
      jitter = 0.;
    }
    return jitter;
  }
}
