package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.feeder.FeederSubsystem;
import frc.team5406.robot.subsystems.gates.BackGateSubsystem;
import frc.team5406.robot.subsystems.gates.FrontGateSubsystem;
import frc.team5406.robot.subsystems.intake.IntakeSubsystem;

public class OuttakeLowerCommand extends CommandBase {
    IntakeSubsystem intake;
    FeederSubsystem feeder;
    FrontGateSubsystem frontGate;
    DoubleSupplier analog;

    public OuttakeLowerCommand(IntakeSubsystem _intake, FeederSubsystem _feeder, FrontGateSubsystem _frontGate) {
        intake = _intake;
        feeder = _feeder;
        frontGate = _frontGate;
        addRequirements(intake, feeder, frontGate);
    }

    

    @Override
    public void execute() {
        intake.intakeExtend();
        feeder.setConveyorBottomSpeed(Constants.CONVEYOR_OUTTAKE_SPEED_BOTTOM);
        feeder.setConveyorTopSpeed(Constants.CONVEYOR_OUTTAKE_SPEED_TOP);
    }

    @Override
    public void initialize() {
        frontGate.frontGateExtend();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Outtake - End");
        intake.intakeRetract();
        feeder.stopConveyorBottom();
        feeder.stopConveyorTop();
        frontGate.frontGateRetract();        
    }
}
