// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetainConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final CANCoder turningEncoder;

  private final RelativeEncoder driveEncoder;

  private final PIDController rotController;

  public SwerveModule(int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannelA, boolean driveInverted) {

    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    turningEncoder = new CANCoder(turningEncoderChannelA);
    turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    driveEncoder = driveMotor.getEncoder();

    driveMotor.setInverted(driveInverted);
    turningMotor.setInverted(false);

    rotController = new PIDController(driveMotorChannel, 0, 0);
    rotController.enableContinuousInput(-180.0, 180.0);
    resetAllEncoder();
    clearSticklyFault();
    stopModule();
  }

  public void resetAllEncoder() {
    driveEncoder.setPosition(0);
  }

  public void clearSticklyFault() {
    driveMotor.clearFaults();
    turningMotor.clearFaults();
  }

  public void stopModule() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  // to get the single swerveModule speed and the turning rate
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveRate(), new Rotation2d(Math.toRadians(turningEncoder.getAbsolutePosition())));
  }

  // to get the drive distance
  public double getDriveDistance() {
    return driveEncoder.getPosition() / 6.75 * 2 * Math.PI * ModuleConstants.kWheelRadius;
  }

  // calculate the rate of the drive
  public double getDriveRate() {
    return driveEncoder.getVelocity() / 60.0 / 6.75 * 2 * Math.PI * ModuleConstants.kWheelRadius;
  }

  // to get rotation of turning motor
  public double getRotation() {
    return turningEncoder.getAbsolutePosition();
  }

  // to the get the postion by wpi function
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistance(), new Rotation2d(Math.toRadians(turningEncoder.getAbsolutePosition())));
  }

  public double[] optimizeOutputVoltage(SwerveModuleState goalState, double currentTurningDegree) {
    goalState = SwerveModuleState.optimize(goalState, Rotation2d.fromDegrees(currentTurningDegree));
    double driveMotorVoltage = ModuleConstants.kDesireSpeedtoMotorVoltage * goalState.speedMetersPerSecond;
    double turningMotorVoltage = rotController.calculate(currentTurningDegree, goalState.angle.getDegrees());
    double[] moduleState = { driveMotorVoltage, turningMotorVoltage };
    return moduleState;
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < DrivetainConstants.kMinJoyStickValue) {
      stopModule();
    } else {
      var moduleState = optimizeOutputVoltage(desiredState, getRotation());
      driveMotor.setVoltage(moduleState[0]);
      turningMotor.setVoltage(moduleState[1]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
