// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SimpleDriveTrainSubsystem;

import java.util.function.Supplier;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArcadeDriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SimpleDriveTrainSubsystem m_simpleDriveSubsystem;
  // run functions and use the return value within each command
  private final Supplier<Double> m_speedFunction, m_turnFunction;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDriveCommand(SimpleDriveTrainSubsystem subsystem, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
    this.m_simpleDriveSubsystem = subsystem;
    this.m_speedFunction = speedFunction;
    this.m_turnFunction = turnFunction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ArcadeDriveCommand started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("ArcadeDriveCommand running!");
    double realTimeSpeed = m_speedFunction.get();
    double realTimeTurn  = m_turnFunction.get();
    
    if (Math.abs(realTimeSpeed) > 0 || Math.abs(realTimeTurn) > 0.) {
      System.out.println(Double.toString(realTimeSpeed) + " / " + Double.toString(realTimeTurn));
    }

    SmartDashboard.putNumber("speed", realTimeSpeed);
    SmartDashboard.putNumber("rotate", realTimeTurn);

    m_simpleDriveSubsystem.arcadeDrive(realTimeSpeed, realTimeTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ArcadeDriveCommand ended!");
    m_simpleDriveSubsystem.stopRobot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
