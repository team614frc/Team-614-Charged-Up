package frc.robot.commands.Autonomous.TimedBasedAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerLoadPathCommand;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCube;

public class Auto6 extends SequentialCommandGroup {
  public Auto6() {
    addCommands(
        new ScoreMidCube(),
        new PathPlannerLoadPathCommand("pathplanner/generatedJSON/RTCMM.wpilib.json", true));
  }
}
