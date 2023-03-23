package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.PIDCommand.TiltPIDCommand;
import frc.robot.commands.SequentialParallelCommands.GroundIntake;
import frc.robot.commands.SequentialParallelCommands.PchooOverCS;
import frc.robot.commands.SequentialParallelCommands.ScoreHighCubeAuto;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCone;

public class getEventMap {

  public getEventMap() {
  }

  public static HashMap<String, Command> ScoreGrab() {
    HashMap<String, Command> ScoreGrabPchooBalance = new HashMap<>();
    ScoreGrabPchooBalance.put("score1", new ScoreHighCubeAuto());
    ScoreGrabPchooBalance.put("intake", new GroundIntake());
    ScoreGrabPchooBalance.put("armup", new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT));
    ScoreGrabPchooBalance.put("score2", new ScoreMidCone());
    return ScoreGrabPchooBalance;
  }

  public static HashMap<String, Command> ScoreRotateBalance() {
    HashMap<String, Command> ScoreRotateBalance = new HashMap<>();
    ScoreRotateBalance.put("score1", new ScoreHighCubeAuto());
    ScoreRotateBalance.put("intake", new GroundIntake());
    ScoreRotateBalance.put("armup", new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT));
    ScoreRotateBalance.put("pchoo", new PchooOverCS());
    ScoreRotateBalance.put("balance", new ForwardBalance());
    return ScoreRotateBalance;
  }

  public static HashMap<String, Command> ScoreAutoBalanceBlue() {
    HashMap<String, Command> ScoreAutoBalanceBlue = new HashMap<>();
    ScoreAutoBalanceBlue.put("balance", new ForwardBalance());
    System.out.print("asdoiasbdioiuabsdub");
    return ScoreAutoBalanceBlue;
  }

  public static HashMap<String, Command> ScoreAutoBalanceRed() {
    HashMap<String, Command> ScoreAutoBalanceRed = new HashMap<>();
    ScoreAutoBalanceRed.put("balance", new ForwardBalance());
    return ScoreAutoBalanceRed;
  }
}
