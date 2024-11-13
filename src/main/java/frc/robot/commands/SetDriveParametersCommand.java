// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SimulationConstants;

public class SetDriveParametersCommand extends Command {
  /** Creates a new SetDriveParametersCommand. */
  private final double forwardSpeed;
  private final double turnSpeed;

  private int debugCounter;

  public SetDriveParametersCommand(double speed, double rotate) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.forwardSpeed = speed;
    this.turnSpeed = rotate;
    //SimulationConstants.simSpeed = forwardSpeed;
    //SimulationConstants.simTurn = turnSpeed;
    System.out.println("Ctor:" + String.valueOf(SimulationConstants.simSpeed) + " | " + String.valueOf(SimulationConstants.simTurn));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SetDriveParametersCommand started!");
    SimulationConstants.simSpeed = forwardSpeed;
    SimulationConstants.simTurn = turnSpeed;
    System.out.println("initialize (local):" + String.valueOf(forwardSpeed) + " / " + String.valueOf(turnSpeed));
    System.out.println("initialize:" + String.valueOf(SimulationConstants.simSpeed) + " / " + String.valueOf(SimulationConstants.simTurn));
    SmartDashboard.putNumber("speed", SimulationConstants.simSpeed);
    SmartDashboard.putNumber("rotate", SimulationConstants.simTurn);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SimulationConstants.simSpeed = forwardSpeed;
    //SimulationConstants.simTurn = turnSpeed;
    if (Constants.debug || debugCounter % 100 == 0) {
      System.out.println("execute (local):" + String.valueOf(forwardSpeed) + " / " + String.valueOf(turnSpeed));
      System.out.println("execute:" + String.valueOf(SimulationConstants.simSpeed) + " / " + String.valueOf(SimulationConstants.simTurn));
    }
    SmartDashboard.putNumber("speed", SimulationConstants.simSpeed);
    SmartDashboard.putNumber("rotate", SimulationConstants.simTurn);

    debugCounter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
