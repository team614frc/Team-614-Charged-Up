package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.PathPlannerCommands.EventMap;
import frc.robot.Commands.PathPlannerCommands.PathPlannerLoadEventMapCommand;
import frc.robot.Commands.SequentialParallelCommands.ScoreMidCubeAuto;

public class BlueScoreGrabAuto extends SequentialCommandGroup {
  public BlueScoreGrabAuto() {
    addCommands(
      new ScoreMidCubeAuto(),
        new PathPlannerLoadEventMapCommand("TopBlueScoreGrab", EventMap.ScoreGrab()).withTimeout(15));
  }
}
