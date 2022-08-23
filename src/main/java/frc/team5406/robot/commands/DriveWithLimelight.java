package frc.team5406.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.team5406.robot.subsystems.LimelightSubsystem;

import frc.team5406.robot.subsystems.drive.DriveSubsystem;


/** A command that will turn the robot to the specified angle. */
public class DriveWithLimelight extends PIDCommand {

  private final DriveSubsystem m_swerve;

  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public DriveWithLimelight(DriveSubsystem drive, LimelightSubsystem limelight, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
      //drive = _drive;
     
    super(
        new PIDController(Constants.LL_TURN_P, Constants.LL_TURN_I, Constants.LL_TURN_D),
        // Close loop on heading
        limelight::getLLtx,
        // center limelight on target, tx = 0
        0,
        // Pipe output to turn robot
        output -> drive.drive(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble(), output, true),
        // Require the drive
        drive);
        m_swerve = drive;

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.LL_TURN_TOLERANCE);
  }

  @Override
  public void end(boolean interrupted) {
      m_swerve.drive(0, 0, 0, true);
  }

}




                


