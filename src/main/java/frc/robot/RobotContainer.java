// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Drivetain;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  Drivetain drivetain;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final PowerDistribution pd = new PowerDistribution();

  private boolean chooseJoy;
  private double[] chassisSpeeds = new double[3];

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    putDashboard();
    getValueFromDashboard();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    drivetain = new Drivetain();
    if (chooseJoy) {
      drivetain.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              () -> drivetain.drive(
                  DrivetainConstants.kMaxSpeed * driverController.getLeftY(),
                  DrivetainConstants.kMaxSpeed * driverController.getLeftX(),
                  DrivetainConstants.kMaxSpeed * driverController.getRightX(),
                  !driverController.getHID().getAButton()),
              drivetain));
    } else {
      drivetain.setDefaultCommand(
          new RunCommand(
              () -> drivetain.drive(
                  chassisSpeeds[0], chassisSpeeds[1], chassisSpeeds[2], !driverController.getHID().getAButton()),
              drivetain));
    }
  }

  private void putDashboard(){
    SmartDashboard.putNumber("xbox_leftX", driverController.getLeftX());
    SmartDashboard.putNumber("xbox_leftY", driverController.getLeftY());
    SmartDashboard.putNumber("pd_voltage", pd.getVoltage());
    SmartDashboard.putBoolean("chooseJoy", chooseJoy);
    SmartDashboard.putNumber("xSpeed", chassisSpeeds[0]);
    SmartDashboard.putNumber("ySpeed", chassisSpeeds[1]);
    SmartDashboard.putNumber("rotSpeed", chassisSpeeds[2]);
  }

  private void getValueFromDashboard(){
    chooseJoy = SmartDashboard.getBoolean("chooseJoy", true);
    chassisSpeeds[0] = SmartDashboard.getNumber("xSpeed", 0);
    chassisSpeeds[1] = SmartDashboard.getNumber("ySpeed", 0);
    chassisSpeeds[2] = SmartDashboard.getNumber("rotSpeed", 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto();
  }
}
