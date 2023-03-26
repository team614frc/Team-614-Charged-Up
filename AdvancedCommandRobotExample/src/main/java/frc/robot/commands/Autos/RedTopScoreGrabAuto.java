package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerCommands.EventMap;
import frc.robot.commands.PathPlannerCommands.PathPlannerLoadEventMapCommand;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCubeAuto;

public class RedTopScoreGrabAuto extends SequentialCommandGroup {
  public RedTopScoreGrabAuto() {
    addCommands(
      new ScoreMidCubeAuto(),
        new PathPlannerLoadEventMapCommand("TopRedScoreGrab", EventMap.ScoreGrab()).withTimeout(15));
  }
}
