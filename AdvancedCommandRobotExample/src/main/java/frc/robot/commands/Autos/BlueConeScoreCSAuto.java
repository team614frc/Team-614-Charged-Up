package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerCommands.EventMap;
import frc.robot.commands.PathPlannerCommands.PathPlannerLoadEventMapCommand;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCone;

public class BlueConeScoreCSAuto extends SequentialCommandGroup {
  public BlueConeScoreCSAuto() {
    addCommands(
      new ScoreMidCone(),
      new PathPlannerLoadEventMapCommand("CuBlueScoreCS", EventMap.ScoreBalance()).withTimeout(15));
  }
}
