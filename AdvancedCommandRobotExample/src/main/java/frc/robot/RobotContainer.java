package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Autos.Auto1;
import frc.robot.Commands.Autos.Auto2;
import frc.robot.Commands.Autos.Auto3;
import frc.robot.Commands.Autos.Auto4;
import frc.robot.Commands.Autos.Auto5;
import frc.robot.Commands.Autos.Auto6;
import frc.robot.Commands.Autos.DoNothingAuto;
import frc.robot.Commands.Autos.RedScoreGrabAuto;
import frc.robot.Commands.Autos.BlueScoreGrabAuto;
import frc.robot.Commands.Autos.RedScoreRotateBalance;
import frc.robot.Commands.Autos.BlueScoreRotateBalance;
import frc.robot.Commands.Autos.TestAuto;
import frc.robot.Commands.PIDCommand.TiltPIDCommand;
import frc.robot.Commands.PathPlannerCommands.ForwardBalance;
import frc.robot.Commands.SequentialParallelCommands.GroundIntake;
import frc.robot.Commands.SequentialParallelCommands.LoadStation;
import frc.robot.Commands.SequentialParallelCommands.PchooOverCS;
import frc.robot.Commands.SequentialParallelCommands.ScoreHighCube;
import frc.robot.Commands.SequentialParallelCommands.ScoreHybrid;
import frc.robot.Commands.SequentialParallelCommands.ScoreMidCone;
import frc.robot.Commands.SequentialParallelCommands.ScoreMidCube;
import frc.robot.Commands.SimpleCommands.ArcadeDrive;
import frc.robot.Commands.SimpleCommands.Extend;
import frc.robot.Commands.SimpleCommands.MaxTiltDown;
import frc.robot.Commands.SimpleCommands.MaxTiltUp;
import frc.robot.Commands.SimpleCommands.Retract;
import frc.robot.Commands.SimpleCommands.SetLEDColorCommand;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.Manipulator;
import frc.robot.Subsystems.TiltSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
    public static CommandXboxController m_CommandXboxController = new CommandXboxController(
            Constants.DRIVER_CONTROLLER_PORT);
    public static CommandXboxController co_CommandXboxController = new CommandXboxController(
            Constants.CO_DRIVER_CONTROLLER_PORT);

    public static Manipulator manipulator = new Manipulator();
    public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public static TiltSubsystem tiltSubsystem = new TiltSubsystem();
    // public static LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public static LEDSubsystem ledSubsystem = new LEDSubsystem();

    private final Command BBCMMCS = new Auto1();
    private final Command BTCMCS = new Auto2();
    private final Command BTCMM = new Auto3();
    private final Command RBCMMCS = new Auto4();
    private final Command RTCMCS = new Auto5();
    private final Command RTCMM = new Auto6();
    private final Command TestAuto = new TestAuto();
    private final Command DoNothing = new DoNothingAuto();
    private final Command ScoreMidCubeAuto = new ScoreMidCube();
    private final Command ScoreHighCubeAuto = new ScoreHighCube();
    private final Command RedScoreGrabAuto = new RedScoreGrabAuto();
    private final Command BlueScoreGrabAuto = new BlueScoreGrabAuto();
    private final Command RedScoreRotateBalance = new RedScoreRotateBalance();
    private final Command BlueScoreRotateBalance = new BlueScoreRotateBalance();


    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        driveTrainSubsystem.setDefaultCommand(new ArcadeDrive());

        m_chooser.addOption("Blue - Mid Cube Over Charge Station #1", BBCMMCS);
        m_chooser.addOption("Blue - Mid Cube Around Charge Station #2", BTCMCS);
        m_chooser.addOption("Blue - Mid Cube Mobility #3", BTCMM);
        m_chooser.addOption("Red - Mid Cube Over Charge Station #4", RBCMMCS);
        m_chooser.addOption("Red - Mid Cube Around Charge Station #5", RTCMCS);
        m_chooser.addOption("Red - Mid Cube Mobility", RTCMM);
        m_chooser.addOption("--TESTAUTO--", TestAuto);
        m_chooser.setDefaultOption("Do Nothing", DoNothing);
        m_chooser.addOption("Score Mid Cube and do Nothing:", ScoreMidCubeAuto);
        m_chooser.addOption("Score High Cube and do Nothing", ScoreHighCubeAuto);
        m_chooser.addOption("Red - Score Grab", RedScoreGrabAuto);
        m_chooser.addOption("Blue - Score Grab", BlueScoreGrabAuto);
        m_chooser.addOption("Red - Score Rotate Balance", RedScoreRotateBalance);
        m_chooser.addOption("Blue - Score Rotate Balance", BlueScoreRotateBalance);

        SmartDashboard.putData(m_chooser);
    }

    private void configureBindings() {
        // MAIN DRIVER CONTROLLER BINDS
        m_CommandXboxController.button(Constants.A_BUTTON).onTrue(new ScoreHybrid());
        m_CommandXboxController.button(Constants.Y_BUTTON).onTrue(new ScoreHighCube());
        m_CommandXboxController.button(Constants.X_BUTTON).onTrue(new ScoreMidCube());
        m_CommandXboxController.button(Constants.B_BUTTON).onTrue(new ScoreMidCone());
        m_CommandXboxController.button(Constants.RIGHT_BUMPER).onTrue(new LoadStation());
        m_CommandXboxController.button(Constants.LEFT_BUMPER).onTrue(new GroundIntake());
        m_CommandXboxController.rightTrigger().onTrue(new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT));
        m_CommandXboxController.leftTrigger().onTrue(new PchooOverCS());
        m_CommandXboxController.button(Constants.START_BUTTON).toggleOnTrue(new
        SetLEDColorCommand(0)); //Sets LED's to purple
        m_CommandXboxController.button(Constants.BACK_BUTTON).toggleOnTrue(new
        SetLEDColorCommand(1)); //Sets LED's to yellow

        // // CO-DRIVER CONTROLLER BINDS
        co_CommandXboxController.button(Constants.A_BUTTON).onTrue(new ScoreHybrid());
        co_CommandXboxController.button(Constants.Y_BUTTON).onTrue(new ScoreHighCube());
        co_CommandXboxController.button(Constants.X_BUTTON).onTrue(new ScoreMidCube());
        co_CommandXboxController.button(Constants.B_BUTTON).onTrue(new ScoreMidCone());
        co_CommandXboxController.button(Constants.RIGHT_BUMPER).onTrue(new LoadStation());
        co_CommandXboxController.button(Constants.LEFT_BUMPER).onTrue(new GroundIntake());
        co_CommandXboxController.rightTrigger().onTrue(new TiltPIDCommand(Constants.TILT_DEFAULT_SETPOINT));
        co_CommandXboxController.leftTrigger().onTrue(new PchooOverCS());
        co_CommandXboxController.axisGreaterThan(1, 0.5).whileTrue(new Retract());
        co_CommandXboxController.axisLessThan(1, -0.5).whileTrue(new Extend());
        co_CommandXboxController.axisGreaterThan(5, 0.5).whileTrue(new MaxTiltDown());
        co_CommandXboxController.axisLessThan(5, -0.5).whileTrue(new MaxTiltUp());
        co_CommandXboxController.button(Constants.START_BUTTON).toggleOnTrue(new
        SetLEDColorCommand(0)); //Sets LED's to purple
        co_CommandXboxController.button(Constants.BACK_BUTTON).toggleOnTrue(new
        SetLEDColorCommand(1)); //Sets LED's to yellow
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
