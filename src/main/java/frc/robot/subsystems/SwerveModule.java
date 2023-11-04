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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetainConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final CANCoder turningEncoder;

  private final RelativeEncoder driveEncoder;

  private String name;

  // private final PIDController drivePIDController = new
  // PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // feedforward sensor-now no use
  // private final SimpleMotorFeedforward driveFeedforward = new
  // SimpleMotorFeedforward(1, 1);;
  // private final SimpleMotorFeedforward turnFeedforward = new
  // SimpleMotorFeedforward(1, 1);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  // the constants need to test
  // private final ProfiledPIDController turningPIDController = new
  // ProfiledPIDController(
  // ModuleConstants.kPModuleTurningController,
  // 0,
  // 0,
  // new TrapezoidProfile.Constraints(
  // ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
  // ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  public SwerveModule(int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannelA, boolean driveInverted, String name) {

    name = this.name;

    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    turningEncoder = new CANCoder(turningEncoderChannelA);
    turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    driveEncoder = driveMotor.getEncoder();
    driveMotor.setCANTimeout(0);
    turningMotor.setCANTimeout(0);
    driveMotor.setInverted(driveInverted);
    turningMotor.setInverted(true);
    resetAllEncoder();
    clearSticklyFault();
    stopModule();
    // turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
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

  // to the get the postion by wpi function
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistance(), new Rotation2d(Math.toRadians(turningEncoder.getAbsolutePosition())));
  }

  // public void setDriveMotorReverse(){
  // driveMotor.setInverted(true);
  // }

  // public void setTurningMotorReverse(){
  // turningMotor.setInverted(true);
  // }

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

  public double checkTuringVoltageOverLimit(double turingDegree) {
    double currentDegree = turingDegree;
    if (currentDegree > ModuleConstants.kMaxSpeedTurningDegree) {
      currentDegree = ModuleConstants.kMaxSpeedTurningDegree;
    } else if (currentDegree < -ModuleConstants.kMaxSpeedTurningDegree) {
      currentDegree = -ModuleConstants.kMaxSpeedTurningDegree;
    }
    double correctVoltage = currentDegree * ModuleConstants.kMaxModuleTuringVoltage
        / ModuleConstants.kMaxSpeedTurningDegree;
    return correctVoltage;
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < DrivetainConstants.kMinSpeed){
      stopModule();
    }else{
      double goalTuringDegree = desiredState.angle.getDegrees();
      double currentTuringDegree = turningEncoder.getAbsolutePosition();
      double error = goalTuringDegree - currentTuringDegree;
      if (error > 180) {
        error = error - 360;
      } else if (error < -180) {
        error = 360 + error;
      }
        driveMotor.setVoltage(ModuleConstants.kDesireSpeedtoMotorVoltage * desiredState.speedMetersPerSecond);
        turningMotor.setVoltage(checkTuringVoltageOverLimit(error));
    
    
    }
    
  }

  // calculate the rate of the drive
  public double getDriveRate() {
    return driveEncoder.getVelocity() / 60.0 / 6.75 * 2 * Math.PI * ModuleConstants.kWheelRadius;
  }

  // for one module test
  public double getRotation() {
    return turningEncoder.getAbsolutePosition();
  }

  // for one module test
  public void setMotorPower(double driveSpd, double rotSpd) {
    driveMotor.set(0.6 * driveSpd);
    turningMotor.set(rotSpd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
