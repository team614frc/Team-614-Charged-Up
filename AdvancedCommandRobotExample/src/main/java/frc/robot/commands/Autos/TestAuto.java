package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerCommands.EventMap;
import frc.robot.commands.PathPlannerCommands.PathPlannerLoadEventMapCommand;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCubeAuto;

public class TestAuto extends SequentialCommandGroup {
  public TestAuto() {
    addCommands(
      new ScoreMidCubeAuto(),
        new PathPlannerLoadEventMapCommand("TopBlueScoreGrab", EventMap.ScoreBalance()).withTimeout(15));
  }
}