package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PIDCommand.DrivePositionPIDCommand;

public class TestAuto extends SequentialCommandGroup {

  public TestAuto() {
    // Auto for testing purposes
    addCommands(new DrivePositionPIDCommand(1));
  }
}
