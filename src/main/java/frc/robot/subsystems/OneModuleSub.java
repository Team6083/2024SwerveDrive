// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OneModuleSub extends SubsystemBase {
  /** Creates a new OneMaduleSub. */

  private final SwerveModule motor;

  private final PIDController rotController = new PIDController(0.005, 0, 0);

  public OneModuleSub() {
    rotController.enableContinuousInput(-180, 180);
    motor = new SwerveModule(10, 11, 5, true);
  }

  public void drive(double rightX, double rightY, double speed) {
    motor.setMotorPower(speed,rightX);
  }

  public double PIDcontrolRot(double rightX, double rightY) {
    if (Math.abs(rightX) > 0.1 || Math.abs(rightY) > 0.1) {
      double angle = Math.atan2(-rightY, rightX);
      if (angle > 3.0*Math.PI / 2.0) {
        angle = angle - 2.0 * Math.PI;
      }
      if(angle > Math.PI){
        angle = -Math.PI+(angle-Math.PI)%Math.PI;
      }
      if(angle<-Math.PI){
        angle = Math.PI-(-Math.PI-angle)%Math.PI;
      }
      rotController.setSetpoint(Math.toDegrees(angle) - 90);
      double rotSpd = rotController.calculate(motor.getRotation());
      return -rotSpd;
    }
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("angle", motor.getRotation());
  }
}
