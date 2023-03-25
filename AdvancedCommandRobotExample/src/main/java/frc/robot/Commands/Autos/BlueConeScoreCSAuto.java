package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.PathPlannerCommands.EventMap;
import frc.robot.Commands.PathPlannerCommands.PathPlannerLoadEventMapCommand;
import frc.robot.Commands.SequentialParallelCommands.ScoreMidCone;
import frc.robot.Commands.SequentialParallelCommands.ScoreMidCubeAuto;

public class BlueConeScoreCSAuto extends SequentialCommandGroup {
  public BlueConeScoreCSAuto() {
    addCommands(
      new ScoreMidCone(),
      new PathPlannerLoadEventMapCommand("CuBlueScoreCS", EventMap.ScoreBalance()).withTimeout(15));
  }
}
