// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.subsystems.SimpleDriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {
  /** Example static factory for an autonomous command. 
  public static Command autoTimerDrive(SimpleDriveTrainSubsystem subsystem) {
    return Commands.sequence(subsystem.tankDrive(0.5, 0.5), new DriveForwardTimerCommand(subsystem, 5., 0.5));
  }
*/

  public static Command autoTimerDrive(SimpleDriveTrainSubsystem subsystem) {
    return Commands.sequence(new DriveForwardTimerCommand(subsystem, SimulationConstants.simTimerAutonoumousAutoInterface, SimulationConstants.simSpeed));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  // some fancy pattern driving
  public static SequentialCommandGroup MoveRotate4Times (SimpleDriveTrainSubsystem subsystem){
    // forward for 5s at 0.5; rotate in place at 0.3; forward for 5s at 0.5; rotate in place at 0.3
    double rotationTime = 0.1;
    SequentialCommandGroup sequence1 = new SequentialCommandGroup(
      new DriveForwardTimerCommand(subsystem, 5., 0.5),
      new DriveArcadeTimerCommand(subsystem, rotationTime, 0., 0.3),
      new DriveForwardTimerCommand(subsystem, 5., 0.5),
      new DriveArcadeTimerCommand(subsystem, rotationTime, 0., 0.3),
      new DriveForwardTimerCommand(subsystem, 5., 0.5),
      new DriveArcadeTimerCommand(subsystem, rotationTime, 0., 0.3),
      new DriveForwardTimerCommand(subsystem, 5., 0.5),
      new DriveArcadeTimerCommand(subsystem, rotationTime, 0., 0.3)
    );
    return sequence1;
  }
}
