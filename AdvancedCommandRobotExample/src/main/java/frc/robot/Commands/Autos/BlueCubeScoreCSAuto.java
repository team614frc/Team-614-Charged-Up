package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.PathPlannerCommands.EventMap;
import frc.robot.Commands.PathPlannerCommands.PathPlannerLoadEventMapCommand;
import frc.robot.Commands.SequentialParallelCommands.ScoreMidCubeAuto;

public class BlueCubeScoreCSAuto extends SequentialCommandGroup {
  public BlueCubeScoreCSAuto() {
    addCommands(
      new ScoreMidCubeAuto(),
      new PathPlannerLoadEventMapCommand("CuBlueScoreCS", EventMap.ScoreBalance()).withTimeout(15));
  }
}
