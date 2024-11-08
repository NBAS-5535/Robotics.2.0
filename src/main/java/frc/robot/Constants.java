// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // which robot is being driven?
  //    1: 5535_PID (Mater); 
  //    2: 5535_Backup
  public static int motorType = 1; 

  public static class MotorSpecs {
    public static int leftFrontMotorDeviceId  = 4;
    public static int leftBackMotorDeviceId   = 3;
    public static int rightFrontMotorDeviceId = 2;
    public static int rightBackMotorDeviceId  = 1;
  }

  public static class MotorSpecs_2 {
    public static int leftFrontMotorDeviceId  = 3;
    public static int leftBackMotorDeviceId   = 2;
    public static int rightFrontMotorDeviceId = 4;
    public static int rightBackMotorDeviceId  = 1;
  }

  public static class MotorSpecs_3 {
    public static int leftFrontMotorDeviceId  = 1;
    public static int leftBackMotorDeviceId   = 2;
    public static int rightFrontMotorDeviceId = 3;
    public static int rightBackMotorDeviceId  = 4;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    // 5535 robot
    public static final int kArcadeDriveSpeedAxis = 1;
    public static final int kArcadeDriveTurnAxis = 0;

    // dead-band for jotstick
    public static final double jitterTolerance = 0.05;
  }

  public static class SimulationConstants {
    public static final boolean isSim = false;
    public static final double simTimer = 5.;
    public static final double simEncoderRevolutions = 50.;
    public static final double simSpeed = 0.4;
    public static final double simTurn = 0.2;
    
    public static final double simTimerAutonoumous = 10.;
    public static final double simTimerAutonoumousAutoInterface = 8.;

  }

  public static class PIDConstants {
    public static final double kpSpeed = 1.0;
    public static final double kiSpeed = 0.;
    public static final double kdSpeed = 0.;

    public static final double kpTurn = 1.0;
    public static final double kiTurn = 0.;
    public static final double kdTurn = 0.;
  }
}
