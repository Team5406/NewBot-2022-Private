// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5406.robot.commands.DefaultDriveCommand;
//import frc.team5406.robot.commands.IntakeCommand;
import frc.team5406.robot.subsystems.drive.DriveSubsystem;
/*import frc.team5406.robot.subsystems.feeder.FeederSubsystem;
import frc.team5406.robot.subsystems.intake.IntakeSubsystem;
import frc.team5406.robot.subsystems.shooter.BoosterSubsystem;
import frc.team5406.robot.subsystems.shooter.FlywheelSubsystem;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;*/
import frc.team5406.robot.triggers.JoystickMoved;
import frc.team5406.robot.triggers.TriggerPressed;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_swerve = new DriveSubsystem();
  /*FeederSubsystem m_feeder = new FeederSubsystem();
  IntakeSubsystem m_intake = new IntakeSubsystem();
  BoosterSubsystem m_booster = new BoosterSubsystem();
  FlywheelSubsystem m_flywheel = new FlywheelSubsystem();
  HoodSubsystem m_hood = new HoodSubsystem();*/

  // The driver's controller
  XboxController operatorGamepad = new XboxController(Constants.OPERATOR_CONTROLLER);
  XboxController driverGamepad = new XboxController(Constants.DRIVER_CONTROLLER);

  Trigger driverLeftTrigger = new TriggerPressed(driverGamepad::getLeftTriggerAxis);
  Trigger driverRightTrigger = new TriggerPressed(driverGamepad::getRightTriggerAxis);
  Trigger operatorLeftTrigger = new TriggerPressed(operatorGamepad::getLeftTriggerAxis);
  Trigger operatorRightTrigger = new TriggerPressed(operatorGamepad::getRightTriggerAxis);
  Trigger operatorJoystickRight = new JoystickMoved(operatorGamepad::getRightY);

  JoystickButton operatorLeftBumper = new JoystickButton(operatorGamepad, Button.kLeftBumper.value);
  JoystickButton operatorRightModifier = new JoystickButton(operatorGamepad, Button.kStart.value);
  JoystickButton operatorLeftModifier = new JoystickButton(operatorGamepad, Button.kBack.value);
  JoystickButton operatorRightBumper = new JoystickButton(operatorGamepad, Button.kRightBumper.value);
  JoystickButton operatorYButton = new JoystickButton(operatorGamepad, Button.kY.value);
  JoystickButton operatorAButton = new JoystickButton(operatorGamepad, Button.kA.value);
  JoystickButton operatorBButton = new JoystickButton(operatorGamepad, Button.kB.value);
  JoystickButton operatorXButton = new JoystickButton(operatorGamepad, Button.kX.value);
  JoystickButton driverLeftBumper = new JoystickButton(driverGamepad, Button.kLeftBumper.value);
  JoystickButton driverRightBumper = new JoystickButton(driverGamepad, Button.kRightBumper.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_swerve.setDefaultCommand(new DefaultDriveCommand(
            m_swerve,
            () -> -modifyAxis(driverGamepad.getRightY()) * Constants.K_MAX_SPEED,
            () -> -modifyAxis(driverGamepad.getRightX()) * Constants.K_MAX_SPEED,
            () -> -modifyAxis(driverGamepad.getLeftX()) * Constants.K_MAX_ANGULAR_SPEED
    ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*driverRightTrigger.whileActiveContinuous(
      new IntakeCommand(m_intake, m_feeder)
    );*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null; //fix once we get autos
  }

  private double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
