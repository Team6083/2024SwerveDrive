// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final class DrivetainConstants {
    public static final double kMaxSpeed = 7; // 7 meters per second
    public static final double kMinSpeed = 0.5;
    public static final double kMaxAngularSpeed = 2.5 * Math.PI; // 1/2 rotation per second

    public static final boolean kFrontLeftDriveMotorInverted = true;
    public static final boolean kFrontRightDriveMotorInverted = false;
    public static final boolean kBackLeftDriveMotorInverted = true;
    public static final boolean kBackRightDriveMotorInverted = false;
  }

  public static final class ModuleConstants {
    public static final double kWheelRadius = 0.046;

    // public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2.5 *
    // Math.PI;
    // public static final double
    // kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2.5 * Math.PI;

    // public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    // public static final double kDriveEncoderDistancePerPulse =
    // // Assumes the encoders are directly mounted on the wheel shafts
    // (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // public static final double kTurningEncoderDistancePerPulse =
    // // Assumes the encoders are on a 1:1 reduction with the module shaft.
    // (2 * Math.PI) / (double) kEncoderCPR;

    // public static final double kPModuleTurningController = 5;

    // public static final double kPModuleDriveController = 6;

    public static final double kMaxModuleDriveVoltage = 12.0;

    public static final double kDesireSpeedtoMotorVoltage = kMaxModuleDriveVoltage / DrivetainConstants.kMaxSpeed;

    public static final double kMaxModuleTuringVoltage = 11.0;

    public static final double kMaxSpeedTurningDegree = 180.0;
  }

}
