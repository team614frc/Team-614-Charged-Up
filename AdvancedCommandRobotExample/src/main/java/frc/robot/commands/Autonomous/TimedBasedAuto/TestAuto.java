package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerLoadEventMapCommand;
import frc.robot.commands.getEventMap;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCubeAuto;

public class TestAuto extends SequentialCommandGroup {
  public TestAuto() {
    addCommands(
      new ScoreMidCubeAuto(),
        new PathPlannerLoadEventMapCommand("BottomBlueScoreGrabPchooBalance", getEventMap.ScoreGrab()).withTimeout(15));
  }
}