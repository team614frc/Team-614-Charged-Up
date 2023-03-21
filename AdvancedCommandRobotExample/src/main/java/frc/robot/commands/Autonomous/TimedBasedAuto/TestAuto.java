package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerLoadEventMapCommand;

public class TestAuto extends SequentialCommandGroup {
  public TestAuto() {
    addCommands(
        new PathPlannerLoadEventMapCommand().withTimeout(15));
  }
}