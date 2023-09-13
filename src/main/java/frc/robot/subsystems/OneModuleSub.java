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
    motor = new SwerveModule(2, 1, 0);
  }

  public void drive(double rightX, double rightY, double speed) {
    motor.setMotorPower(speed, rightX);
  }

  public double PIDcontrolRot(double rightX, double rightY) {
    double angle = Math.atan2(rightY, rightX);
    if(angle<=Math.PI*3.0/2.0){
      angle -= Math.PI/2.0;
    }
    if(angle>Math.PI*3.0/2.0){
      angle = angle-5.0*Math.PI/2.0;
    }
    double rotSpd = rotController.calculate(motor.getRotation(), Math.toDegrees(angle));
    return rotSpd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
