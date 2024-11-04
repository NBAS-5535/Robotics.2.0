// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SimpleDriveTrainSubsystem;

public class DriveForwardTimerCommand extends Command {
  /** Creates a new DriveForwardTimerCommand. */
  private final SimpleDriveTrainSubsystem simpleDriveTrainSubsystem;
  private final double time;
  private final double speed;
  private double timeToStop;

  public DriveForwardTimerCommand(SimpleDriveTrainSubsystem driveSubsystem, double time, double speed) {
      this.simpleDriveTrainSubsystem = driveSubsystem;
      this.speed = speed;
      this.time = time;
      
      addRequirements(driveSubsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveForwardTimerCommand started!");
    System.out.print(Timer.getFPGATimestamp());
    System.out.print(" - command duration: ");
    System.out.println(time);

    // actual time the timer will end
    timeToStop = Timer.getFPGATimestamp() + time;
    SmartDashboard.putNumber("timeToStop", timeToStop);
    
    System.out.print("timeToStop: ");
    System.out.println(timeToStop);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ( 1 == 0 ) { 
      System.out.println("DriveForwardTimerCommand running!");
      System.out.println(Timer.getFPGATimestamp());
    };
    // robot is moved until timer expires via isTimedOut() method
    simpleDriveTrainSubsystem.tankDrive(speed, speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveForwardTimerCommand ended!");
    SmartDashboard.putNumber("currentTime", Timer.getFPGATimestamp());
    simpleDriveTrainSubsystem.stopRobot();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isTimedOut();
  }

  private boolean isTimedOut(){
    return Timer.getFPGATimestamp() >= timeToStop;
  }
}
