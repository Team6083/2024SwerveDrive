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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetainConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final CANCoder turningEncoder;

  private final RelativeEncoder driveEncoder;

  private final PIDController drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  private final double kModuleMaxAngularVelocity;

  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 1);;
  private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 1);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  public SwerveModule(int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannelA) {

    kModuleMaxAngularVelocity = DrivetainConstants.kMaxAngularSpeed;
    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    turningEncoder = new CANCoder(turningEncoderChannelA);
    turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    driveEncoder = driveMotor.getEncoder();

    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity() / 60.0, new Rotation2d(Math.toRadians(turningEncoder.getAbsolutePosition())));
  }

  public double getDriveDistance() {
    return driveEncoder.getPosition() / 6.75 * 2 * Math.PI * ModuleConstants.kWheelRadius;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistance(), new Rotation2d(Math.toRadians(turningEncoder.getAbsolutePosition())));
  }

  // for one module test
  public double getRotation() {
    return turningEncoder.getAbsolutePosition();
  }

  public void setMotorPower(double driveSpd, double rotSpd) {
    driveMotor.set(0.6*driveSpd);
    driveMotor.set(rotSpd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d());

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity() / 60.0,
        state.speedMetersPerSecond);

    final double kdriveFeedforward = driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = turningPIDController.calculate(turningEncoder.getPosition(), state.angle.getRadians());

    final double kturnFeedforward = turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

    driveMotor.setVoltage(driveOutput + kdriveFeedforward);
    turningMotor.setVoltage(turnOutput + kturnFeedforward);
  }
}
