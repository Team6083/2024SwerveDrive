// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetainConstants;

public class Drivetain extends SubsystemBase {
  /** Creates a new Drivetain. */
  private final Translation2d frontLeftLocation;
  private final Translation2d frontRightLocation;
  private final Translation2d backLeftLocation;
  private final Translation2d backRightLocation;

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  private final AHRS gyro;

  private final PIDController rotController = new PIDController(0.5, 0, 0);

  // choose oneMuduleTest
  private boolean oneMuduleTest;
  private double chooseModule;
  private double moduleSpeed;
  private double moduleDegree;

  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

  public Drivetain() {
    frontLeftLocation = new Translation2d(0.3, 0.3);
    frontRightLocation = new Translation2d(0.3, -0.3);
    backLeftLocation = new Translation2d(-0.3, 0.3);
    backRightLocation = new Translation2d(-0.3, -0.3);

    frontLeft = new SwerveModule(10, 11, 5, DrivetainConstants.kFrontLeftDriveMotorInverted);
    frontRight = new SwerveModule(15, 14, 4, DrivetainConstants.kFrontRightDriveMotorInverted);
    backLeft = new SwerveModule(12, 13, 2, DrivetainConstants.kBackLeftDriveMotorInverted);
    backRight = new SwerveModule(17, 16, 3, DrivetainConstants.kBackRightDriveMotorInverted);

    SmartDashboard.putData("frontLeft", frontLeft);
    SmartDashboard.putData("frontRight", frontRight);
    SmartDashboard.putData("backLeft", backLeft);
    SmartDashboard.putData("backRight", backRight);

    gyro = new AHRS(Port.kMXP);

    kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    // create the odometry
    odometry = new SwerveDriveOdometry(
        kinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });

    // reset the gyro
    setGyroReset();

    // set the swerve speed equal 0
    drive(0, 0, 0, false);
  }

  public void setGyroReset() {
    gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * 
   *                      using the wpi function to set the speed of the swerve
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetainConstants.kMaxSpeed);
    if (oneMuduleTest) {
      setOneModuleSpeed((int) chooseModule);
    } else {
      frontLeft.setDesiredState(swerveModuleStates[0]);
      frontRight.setDesiredState(swerveModuleStates[1]);
      backLeft.setDesiredState(swerveModuleStates[2]);
      backRight.setDesiredState(swerveModuleStates[3]);
    }
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
  }

  // by test the different xboxController mode
  public double PIDcontrolRot(double rightX, double rightY) {
    double angle = Math.atan2(rightY, rightX);
    double rotSpd = rotController.calculate(gyro.getRotation2d().getRadians(), angle);
    return rotSpd;
  }

  public void setOneModuleSpeed(int chooseModule) {
    switch (chooseModule) {
      case 0:
        frontLeft.setDesiredState(new SwerveModuleState(moduleSpeed, Rotation2d.fromDegrees(moduleDegree)));
        break;
      case 1:
        frontRight.setDesiredState(new SwerveModuleState(moduleSpeed, Rotation2d.fromDegrees(moduleDegree)));
        break;
      case 2:
        backLeft.setDesiredState(new SwerveModuleState(moduleSpeed, Rotation2d.fromDegrees(moduleDegree)));
        break;
      case 3:
        backRight.setDesiredState(new SwerveModuleState(moduleSpeed, Rotation2d.fromDegrees(moduleDegree)));
        break;
      default:
    }
  }

  public void putDashboard() {
    SmartDashboard.putBoolean("oneMuduleTest", oneMuduleTest);
    SmartDashboard.putNumber("chooseModule", chooseModule);
    SmartDashboard.putNumber("frontLeft_speed", swerveModuleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("frontRight_speed", swerveModuleStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("backLeft_speed", swerveModuleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("backRight_speed", swerveModuleStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("gyro_heading", gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("moduleSpeed", moduleSpeed);
    SmartDashboard.putNumber("moduleDegree", moduleDegree);
  }

  public void getValueFromDashboard() {
    oneMuduleTest = SmartDashboard.getBoolean("oneMuduleTest", false);
    chooseModule = SmartDashboard.getNumber("chooseModule", chooseModule);
    moduleSpeed = SmartDashboard.getNumber("moduleSpeed", moduleSpeed);
    moduleDegree = SmartDashboard.getNumber("moduleDegree", moduleDegree);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    putDashboard();
    getValueFromDashboard();
  }
}
