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
  public static boolean debug = false;
  // which robot is being driven?
  //    1: 5535_PID (Mater); 
  //    2: 5535_Backup
  //    3: TalonFX
  public static int motorType = 1; 
  public static final boolean isSim = true;
  public static final boolean isDemo = true;

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
    public static final double simTimer = 5.;
    public static final double simEncoderRevolutions = 50.;
    public static double simSpeed = 0.7;
    public static double simTurn = 0.3;
    
    public static final double simTimerAutonoumous = 10.;
    public static final double simTimerAutonoumousAutoInterface = 8.;

  }

  public static double storeSpeed;
  
  public static class DriveTrainConstants{
    public static final double kTrackWidth = 27.; //inches
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

/*
      // Default Values 
      private static final double DEFAULT_THUMBSTICK_DEADZONE = 0.1;  // Jiggle room for the thumbsticks
      private static final double DEFAULT_TRIGGER_DEADZONE    = 0.01; // Jiggle room for the triggers
      private static final double DEFAULT_TRIGGER_SENSITIVITY = 0.6;  // If the trigger is beyond this limit, say it has been pressed
      
      // Button Mappings
      private static final int    A_BUTTON_ID                 = 1;
      private static final int    B_BUTTON_ID                 = 2;
      private static final int    X_BUTTON_ID                 = 3;
      private static final int    Y_BUTTON_ID                 = 4;
      private static final int    LB_BUTTON_ID                = 5;
      private static final int    RB_BUTTON_ID                = 6;
      private static final int    BACK_BUTTON_ID              = 7;
      private static final int    START_BUTTON_ID             = 8;
      private static final int    LEFT_THUMBSTIKC_BUTTON_ID   = 9;
      private static final int    RIGHT_THUMBSTICK_BUTTON_ID  = 10;
      
      // Axis Mappings 
      private static final int    LEFT_THUMBSTICK_X_AXIS_ID   = 0;
      private static final int    LEFT_THUMBSTICK_Y_AXIS_ID   = 1;
      private static final int    LEFT_TRIGGER_AXIS_ID        = 2;
      private static final int    RIGHT_TRIGGER_AXIS_ID       = 3;
      private static final int    RIGHT_THUMBSTICK_X_AXIS_ID  = 4;
      private static final int    RIGHT_THUMBSTICK_Y_AXIS_ID  = 5;
      
      
      
      // Instance Values
      private final   int             port;
      private final   Joystick        controller;
      
      public final    Thumbstick      leftStick;
      public final    Thumbstick      rightStick;
      public final    Trigger         lt;
      public final    Trigger         rt;
      public final    DirectionalPad  dPad;
      public final    Button          a;
      public final    Button          b;
      public final    Button          x;
      public final    Button          y;
      public final    Button          lb;
      public final    Button          rb;
      public final    Button          back;
      public final    Button          start;
  
*/