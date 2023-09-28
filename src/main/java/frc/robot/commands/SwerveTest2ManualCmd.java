// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.OneModuleSub;

public class SwerveTest2ManualCmd extends CommandBase {
  /** Creates a new SwerveTest2ManualCmd. */

  private final OneModuleSub oneModuleSub;
  private final CommandXboxController main;

  public SwerveTest2ManualCmd(OneModuleSub oneModuleSub, CommandXboxController main) {
    this.oneModuleSub = oneModuleSub;
    this.main = main;
    addRequirements(this.oneModuleSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    oneModuleSub.drive(0.2*main.getRightX(), main.getRightY(), main.getLeftY());
    double angle = Math.atan2(main.getRightY(), main.getRightX());
    if(angle>3.0*Math.PI/2.0){
      angle = angle - 2.0*Math.PI;
    }
    SmartDashboard.putNumber("controller_turningAngle", Math.toDegrees(angle)-90);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
