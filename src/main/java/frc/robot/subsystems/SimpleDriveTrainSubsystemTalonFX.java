// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorSpecs;
import frc.robot.Constants.MotorSpecs_3;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.SimulationConstants;

public class SimpleDriveTrainSubsystemTalonFX extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
// define motors on the root
  //TalonFX motors
  private TalonFX leftFrontMotor_TalonFX;
  private TalonFX leftBackMotor_TalonFX;
  private TalonFX rightFrontMotor_TalonFX;
  private TalonFX rightBackMotor_TalonFX;

  // get encoders for left/right front motors
  

  // fake encoder increments for simulations
  private int encoderCounter = 0;

  // use differential drive to have coordinated motor movement
  private DifferentialDrive differentialDrive;
   
    //private final SparkPIDController speedPIDController;
    
  
  public SimpleDriveTrainSubsystemTalonFX() {
    this.leftFrontMotor_TalonFX  = new TalonFX(MotorSpecs_3.leftFrontMotorDeviceId);
    this.leftBackMotor_TalonFX   = new TalonFX(MotorSpecs_3.leftBackMotorDeviceId);
    this.rightFrontMotor_TalonFX = new TalonFX(MotorSpecs_3.rightFrontMotorDeviceId);
    this.rightBackMotor_TalonFX  = new TalonFX(MotorSpecs_3.rightBackMotorDeviceId);

    // make sure back follows front (replacement for the deprecated MotorControllerGroup)
    //this.leftBackMotor_TalonFX.follow(leftFrontMotor_TalonFX);
    //this.rightBackMotor.follow(rightFrontMotor);

    // one side need to be inverted; NOTE
    this.leftFrontMotor_TalonFX.setInverted(false);
    this.leftBackMotor_TalonFX.setInverted(false);
    this.rightFrontMotor_TalonFX.setInverted(true);
    this.rightBackMotor_TalonFX.setInverted(true);

    //this.leftFrontEncoder = this.leftFrontMotor.getEncoder();
    //this.rightFrontEncoder = this.rightFrontMotor.getEncoder();
    // reset encoder values to zero
    this.leftFrontMotor_TalonFX.setPosition(0);
    this.rightFrontMotor_TalonFX.setPosition(0);

    this.differentialDrive = new DifferentialDrive(this.leftFrontMotor_TalonFX, this.rightFrontMotor_TalonFX);

    //speedPIDController = this.leftFrontMotor_TalonFX.getPIDController();
    //speedPIDController.setFeedbackDevice(leftFrontEncoder);
    //speedPIDController.setP(PIDConstants.kpSpeed);
    //speedPIDController.setI(PIDConstants.kiSpeed);
    //speedPIDController.setD(PIDConstants.kdSpeed);
    

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
    return this.leftFrontMotor_TalonFX.getPosition().getValue().doubleValue();
  }

  public double getRightFrontEncoderValue(){
    return this.rightFrontMotor_TalonFX.getPosition().getValue().doubleValue();
  }

  public double getAverageEncoderValue(){
    if (Constants.isSim) {
      return encoderCounter;
    } else {      
      return 0.5 * ( leftFrontMotor_TalonFX.getPosition().getValue().doubleValue() + 
                     rightFrontMotor_TalonFX.getPosition().getValue().doubleValue());
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

  }
}
