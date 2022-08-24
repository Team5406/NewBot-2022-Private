// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5406.robot.autos.DriveStraight;
import frc.team5406.robot.autos.FiveBall;
import frc.team5406.robot.autos.RotateStraight;
import frc.team5406.robot.autos.TwoBall;
import frc.team5406.robot.commands.DefaultDriveCommand;
import frc.team5406.robot.commands.AlignWithLimelight;
import frc.team5406.robot.commands.DriveWithLimelight;
import frc.team5406.robot.commands.TurnToAngle;
import frc.team5406.robot.commands.SetShooter;
import frc.team5406.robot.commands.Shoot;
import frc.team5406.robot.subsystems.LimelightSubsystem;
//import frc.team5406.robot.commands.IntakeCommand;
//import frc.team5406.robot.commands.OuttakeLowerCommand;
import frc.team5406.robot.subsystems.drive.DriveSubsystem;
import frc.team5406.robot.subsystems.feeder.FeederSubsystem;
import frc.team5406.robot.subsystems.gates.BackGateSubsystem;
import frc.team5406.robot.subsystems.gates.FrontGateSubsystem;
import frc.team5406.robot.subsystems.intake.IntakeSubsystem;
import frc.team5406.robot.subsystems.shooter.BoosterSubsystem;
import frc.team5406.robot.subsystems.shooter.FlywheelSubsystem;
import frc.team5406.robot.subsystems.shooter.HoodSubsystem;
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
  FeederSubsystem m_feeder = new FeederSubsystem();
  IntakeSubsystem m_intake = new IntakeSubsystem();
  BoosterSubsystem m_booster = new BoosterSubsystem();
  FlywheelSubsystem m_flywheel = new FlywheelSubsystem();
  HoodSubsystem m_hood = new HoodSubsystem();
  FrontGateSubsystem m_frontGate = new FrontGateSubsystem();
  BackGateSubsystem m_backGate = new BackGateSubsystem();
  LimelightSubsystem m_limelight = new LimelightSubsystem();

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
  JoystickButton driverRightModifier = new JoystickButton(driverGamepad, Button.kStart.value);
  JoystickButton driverLeftModifier = new JoystickButton(driverGamepad, Button.kBack.value);
  JoystickButton driverRightJoystickButton = new JoystickButton(driverGamepad, Button.kRightStick.value);
  JoystickButton driverLeftJoystickButton = new JoystickButton(driverGamepad, Button.kLeftStick.value);
  JoystickButton operatorRightBumper = new JoystickButton(operatorGamepad, Button.kRightBumper.value);
  JoystickButton operatorYButton = new JoystickButton(operatorGamepad, Button.kY.value);
  JoystickButton operatorAButton = new JoystickButton(operatorGamepad, Button.kA.value);
  JoystickButton operatorBButton = new JoystickButton(operatorGamepad, Button.kB.value);
  JoystickButton driverXButton = new JoystickButton(operatorGamepad, Button.kX.value);
  JoystickButton driverAButton = new JoystickButton(driverGamepad, Button.kA.value);
  JoystickButton driverBButton = new JoystickButton(driverGamepad, Button.kB.value);
  JoystickButton driverLeftBumper = new JoystickButton(driverGamepad, Button.kLeftBumper.value);
  JoystickButton driverRightBumper = new JoystickButton(driverGamepad, Button.kRightBumper.value);

  DriveStraight driveStraight = new DriveStraight(m_swerve);
  RotateStraight rotateStraight = new RotateStraight(m_swerve);
  TwoBall twoBall = new TwoBall(m_swerve, m_feeder, m_intake, m_booster, m_flywheel, m_hood, m_frontGate, m_backGate, m_limelight);
  FiveBall fiveBall = new FiveBall(m_swerve, m_feeder, m_intake, m_booster, m_flywheel, m_hood, m_frontGate, m_backGate, m_limelight);

    // A chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();
    
    
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    m_swerve.setDefaultCommand(new DefaultDriveCommand(
            m_swerve,
            () -> -modifyAxis(driverGamepad.getLeftY()) * Constants.K_MAX_SPEED,
            () -> -modifyAxis(driverGamepad.getLeftX()) * Constants.K_MAX_SPEED,
            () -> -modifyAxis(driverGamepad.getRightX()) * Constants.K_MAX_ANGULAR_SPEED,
            () -> driverGamepad.getRightBumper()
    ));

        // Add commands to the autonomous command chooser
        m_chooser.setDefaultOption("Drive Straight Auto", driveStraight.getAutonomousCommand());
       // m_chooser.addOption("OneBall", oneBall.getAutonomousCommand());
        m_chooser.addOption("TwoBall", twoBall.getAutonomousCommand());
        m_chooser.addOption("FiveBallRight", fiveBall.getAutonomousCommand());
    
        // Put the chooser on the dashboard
        SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   /* driverLeftTrigger.whileActiveContinuous(
      new IntakeCommand(m_intake, m_feeder)
    );*/

    /*driverAButton.whileActiveContinuous(
      new OuttakeLowerCommand(m_intake, m_feeder)
    );*/
     /*  driverLeftTrigger.whileActiveContinuous(
      new SetShooter(m_flywheel, m_hood, m_limelight)
    );

    driverXButton.whileActiveContinuous(
      new Shoot(m_booster, m_feeder)
    );*/

    driverRightTrigger.whileActiveContinuous(
      new DriveWithLimelight(m_swerve, m_limelight,
      () -> -modifyAxis(driverGamepad.getLeftY()) * Constants.K_MAX_SPEED,
      () -> -modifyAxis(driverGamepad.getLeftX()) * Constants.K_MAX_SPEED
    ));

    driverBButton.whileActiveContinuous(
      new TurnToAngle(45, m_swerve)
      );

    driverLeftModifier.whenActive(m_swerve::zeroGyroscope);
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
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
