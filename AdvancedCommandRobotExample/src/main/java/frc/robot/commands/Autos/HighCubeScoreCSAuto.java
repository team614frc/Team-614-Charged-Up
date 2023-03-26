package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerCommands.EventMap;
import frc.robot.commands.PathPlannerCommands.PathPlannerLoadEventMapCommand;
import frc.robot.commands.SequentialParallelCommands.ScoreHighCubeAuto;

public class HighCubeScoreCSAuto extends SequentialCommandGroup {
  public HighCubeScoreCSAuto() {
    addCommands(
      new ScoreHighCubeAuto(),
      new PathPlannerLoadEventMapCommand("CuBlueScoreCS", EventMap.ScoreBalance()).withTimeout(15));
  }
}
