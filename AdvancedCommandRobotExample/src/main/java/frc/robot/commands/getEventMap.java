package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.PIDCommand.TiltPIDCommand;
import frc.robot.commands.SequentialParallelCommands.GroundIntake;
import frc.robot.commands.SequentialParallelCommands.PchooOverCS;
import frc.robot.commands.SequentialParallelCommands.PchooOverCSBalance;
import frc.robot.commands.SequentialParallelCommands.ScoreHighCubeAuto;
import frc.robot.commands.SequentialParallelCommands.ScoreMidCone;

public class getEventMap {

  public getEventMap() {
  }

  public static HashMap<String, Command> ScoreGrab() {
    HashMap<String, Command> ScoreGrab = new HashMap<>();
    ScoreGrab.put("intake", new GroundIntake());
    ScoreGrab.put("armup", new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT));
    ScoreGrab.put("score2", new ScoreMidCone());
    return ScoreGrab;
  }

  public static HashMap<String, Command> ScoreRotateBalance() {
    HashMap<String, Command> ScoreRotateBalance = new HashMap<>();
    ScoreRotateBalance.put("intake", new GroundIntake());
    ScoreRotateBalance.put("armup", new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT));
    ScoreRotateBalance.put("pchoobalance", new PchooOverCSBalance());
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
    ScoreAutoBalanceRed.put("pathplannersucks", new TiltPIDCommand(9));
    return ScoreAutoBalanceRed;
  }
}
