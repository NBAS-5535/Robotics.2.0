// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SimpleDriveTrainSubsystem;

public class TankDriveCommand extends Command {
  /** Creates a new TankDriveCommand. */
  private final SimpleDriveTrainSubsystem m_simpleDriveSubsystem;
  // run functions and use the return value within each command
  private final Supplier<Double> m_leftMotorSpeedFunction, m_rightMotorSpeedFunction;

  public TankDriveCommand(SimpleDriveTrainSubsystem subsystem,
                          Supplier<Double> leftMotorSpeedFunction, 
                          Supplier<Double> rightMotorSpeedFunction) {
    this.m_simpleDriveSubsystem = subsystem;
    this.m_leftMotorSpeedFunction = leftMotorSpeedFunction;
    this.m_rightMotorSpeedFunction = rightMotorSpeedFunction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    System.out.println("TankDriveCommand running!");
    double rightSpeed = m_leftMotorSpeedFunction.get();
    double leftSpeed  = m_rightMotorSpeedFunction.get();

    if (Constants.debug) m_simpleDriveSubsystem.tankDrive(rightSpeed, leftSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("TankDriveCommand ended!");
    m_simpleDriveSubsystem.stopRobot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
