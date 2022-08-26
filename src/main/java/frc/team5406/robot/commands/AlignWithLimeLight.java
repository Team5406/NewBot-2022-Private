package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.team5406.robot.subsystems.LimelightSubsystem;

import frc.team5406.robot.subsystems.drive.DriveSubsystem;



/** A command that will turn the robot to the specified angle. */
public class AlignWithLimelight extends PIDCommand {
    //DriveSubsystem drive;
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public AlignWithLimelight(DriveSubsystem drive, LimelightSubsystem limelight) {
      //drive = _drive;
    super(
        new PIDController(Constants.LL_TURN_P, Constants.LL_TURN_I, Constants.LL_TURN_D),
        // Close loop on heading
        limelight::getLLtx,
        // center limelight on target, tx = 0
        0,
        // Pipe output to turn robot
        output -> drive.setTurnStates(output),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.LL_TURN_TOLERANCE);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}