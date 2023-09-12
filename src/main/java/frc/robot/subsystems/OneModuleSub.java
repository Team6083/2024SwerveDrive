// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OneModuleSub extends SubsystemBase {
  /** Creates a new OneMaduleSub. */

  private final SwerveModule motor;

  private final PIDController rotController = new PIDController(0.05, 0, 0);

  public OneModuleSub() {
    motor = new SwerveModule(1, 2, 0);
  }

  public void drive(double rightX, double rightY, double speed) {
    motor.setMotorPower(speed, PIDcontrolRot(rightX, rightY));
  }

  public double PIDcontrolRot(double rightX, double rightY) {
    double angle = Math.atan2(rightY, rightX);
    double rotSpd = rotController.calculate(motor.getRotation(), angle);
    return rotSpd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
