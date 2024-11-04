// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SimpleDriveTrainSubsystem;

public class DriveForwardCommand extends Command {
  /** Creates a new DriveForwardCommand. */
  private final SimpleDriveTrainSubsystem simpleDriveTrainSubsystem;
  private final double distance;
  private final double speed;

  private double distanceToStop;

  public DriveForwardCommand(SimpleDriveTrainSubsystem driveSubsystem, double distance, double speed) {
      this.simpleDriveTrainSubsystem = driveSubsystem;
      this.speed = speed;
      this.distance = distance;

      addRequirements(driveSubsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveForwardCommand started!");
    distanceToStop = simpleDriveTrainSubsystem.getAverageEncoderValue() + distance;
    System.out.print("total distance to travel: ");
    System.out.println(distanceToStop);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("DriveForwardCommand running!");
    System.out.println(simpleDriveTrainSubsystem.getAverageEncoderValue());
    // stop the robot if distance = 0
    simpleDriveTrainSubsystem.tankDrive(speed, speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveForwardCommand ended!");
    SmartDashboard.putNumber("endEncoderValue", simpleDriveTrainSubsystem.getAverageEncoderValue());
    simpleDriveTrainSubsystem.stopRobot();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isTimedOut();
  }

  private boolean isTimedOut(){
    return simpleDriveTrainSubsystem.getAverageEncoderValue() - distanceToStop >= 1;
  }
}
