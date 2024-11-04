// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorSpecs;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SimulationConstants;

public class SimpleDriveTrainSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
// define motors on the root
  private CANSparkMax leftFrontMotor  = new CANSparkMax(MotorSpecs.leftFrontMotorDeviceId, MotorType.kBrushless);
  private CANSparkMax leftBackMotor   = new CANSparkMax(MotorSpecs.leftBackMotorDeviceId, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(MotorSpecs.rightFrontMotorDeviceId, MotorType.kBrushless);
  private CANSparkMax rightBackMotor  = new CANSparkMax(MotorSpecs.rightBackMotorDeviceId, MotorType.kBrushless);

  // get encoders for left/right front motors
  private RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  private RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();

  // fake encoder increments for simulations
  private int encoderCounter = 0;

  // use differential drive to have coordinated motor movement
  private DifferentialDrive differentialDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

  private final SparkPIDController speedPIDController;
  
  public SimpleDriveTrainSubsystem() {
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();

    // make sure back follows front (replacement for the deprecated MotorControllerGroup)
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

    // one side need to be inverted; NOTE
    rightFrontMotor.setInverted(true);
    rightBackMotor.setInverted(true);

    // reset encoder values to zero
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);

    speedPIDController = leftFrontMotor.getPIDController();
    speedPIDController.setFeedbackDevice(leftFrontEncoder);
    speedPIDController.setP(PIDConstants.kpSpeed);
    speedPIDController.setI(PIDConstants.kiSpeed);
    speedPIDController.setD(PIDConstants.kdSpeed);
    

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("leftFrontEncoderValue", getLeftFrontEncoderValue());
    SmartDashboard.putNumber("rightFrontEncoderValue", getRightFrontEncoderValue());
    SmartDashboard.putNumber("averageEncoderValue", getAverageEncoderValue());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    encoderCounter += 10;
  }

  // user-defined methods
  // get encoder values
  public double getLeftFrontEncoderValue(){
    return leftFrontEncoder.getPosition();
  }

  public double getRightFrontEncoderValue(){
    return rightFrontEncoder.getPosition();
  }

  public double getAverageEncoderValue(){
    if (SimulationConstants.isSim) {
      return encoderCounter;
    } else {
      return 0.5 * ( leftFrontEncoder.getPosition() + rightFrontEncoder.getPosition());
    }
  }

  // move the robot in arcade/tank modes
  public void arcadeDrive(double forward, double turn){
    SmartDashboard.putString("Motor state", "arcadeDrive");
    differentialDrive.arcadeDrive(forward, turn);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    SmartDashboard.putString("Motor state", "tankDrive");
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void stopRobot(){
    differentialDrive.stopMotor();
    //SmartDashboard.putString("Motor state", "Stopped");
  }

  public void resetEncoders() {
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
  }
}