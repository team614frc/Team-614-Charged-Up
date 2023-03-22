package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.SimpleCommands.Intake;

public class getEventMap {

  public getEventMap() {
  }

  public static HashMap<String, Command> ScoreGrabBalance() {
    HashMap<String, Command> ScoreGrabBalance = new HashMap<>();
    ScoreGrabBalance.put("intake", new Intake(Constants.MANIPULATOR_SPEED_INTAKE));
    ScoreGrabBalance.put("balance", new ForwardBalance());
    return ScoreGrabBalance;
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
