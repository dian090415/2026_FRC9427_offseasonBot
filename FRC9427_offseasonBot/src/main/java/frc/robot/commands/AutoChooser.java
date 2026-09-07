package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.opencv.ml.RTrees;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.RobotStatus.RobotStatus;

public class AutoChooser {
    private final CommandSwerveDrivetrain drive;
    private final superstructure superstructure;

    public final SendableChooser<AutoStart> AutoStartChooser = new SendableChooser<>();
    public final SendableChooser<Sec> SecChooser = new SendableChooser<>();


    public AutoChooser(CommandSwerveDrivetrain drive, superstructure superstructure) {
        this.drive = drive;
        this.superstructure = superstructure;

        this.configureAutoChoosers();
        this.SetNamedCommands();
    }

    public void SetNamedCommands() {
        NamedCommands.registerCommand("shoot", superstructure.autoshoot());
        NamedCommands.registerCommand("stopshoot", superstructure.shootstop());
        NamedCommands.registerCommand("intake", superstructure.intakedown());
        NamedCommands.registerCommand("stopintake", superstructure.intakestop());
    }

    public enum AutoStart {
        LEFT, RIGHT, NONE
    }

    public enum Sec {
        Cooperate,
        No_Cooperate,
    }

    public void configureAutoChoosers() {

        AutoStartChooser.setDefaultOption("None", AutoStart.NONE);
        AutoStartChooser.addOption("Start: Left", AutoStart.LEFT);
        AutoStartChooser.addOption("Start: Right", AutoStart.RIGHT);

        SecChooser.setDefaultOption("Sec_Round/Cooperate", Sec.Cooperate);
        SecChooser.addOption("Sec_Round/No_Cooperate", Sec.No_Cooperate);

        SmartDashboard.putData("Auto/Start Position", AutoStartChooser);
        SmartDashboard.putData("Auto/Sec_Round", SecChooser);
    }

    public Command auto() {
        AutoStart startPose = AutoStartChooser.getSelected();
        Sec SecChoose = SecChooser.getSelected();

        // 1. 增加 Null 防護，避免儀表板未同步導致 Crash
        if (startPose == null)
            startPose = AutoStart.NONE;
        if (SecChoose == null)
            SecChoose = Sec.Cooperate;

        if (startPose == AutoStart.NONE) {

            Pose2d currentPose = this.drive.getPose2d();

            try {
                Pose2d leftStart = PathPlannerPath.fromChoreoTrajectory("Left_Frist_Round")
                        .getStartingHolonomicPose()
                        .orElse(new Pose2d());

                Pose2d rightStart = PathPlannerPath.fromChoreoTrajectory("Right_Frist_Round")
                        .getStartingHolonomicPose()
                        .orElse(new Pose2d());

                // C. 計算距離 (使用 getTranslation().getDistance())
                double distLeft = currentPose.getTranslation().getDistance(leftStart.getTranslation());
                double distRight = currentPose.getTranslation().getDistance(rightStart.getTranslation());

                // D. 比較誰最近
                if (distLeft < distRight) {
                    startPose = AutoStart.LEFT;
                } else {
                    startPose = AutoStart.RIGHT;
                }

            } catch (Exception e) {
                e.printStackTrace();
                startPose = AutoStart.RIGHT;
            }
        }

        // --- 組合路徑邏輯 ---
        Command start = Commands.none();
        switch (startPose) {
            case LEFT:
                start = new PathPlannerAuto("Left_Start_Auto");
                break;
            case RIGHT:
                start = new PathPlannerAuto("Right_Start_Auto");
                break;
            default:
                break;
        }

        Command Sec = Commands.none();
        switch (SecChoose) {
            case Cooperate:
                if (startPose == AutoStart.LEFT)
                    Sec = new PathPlannerAuto("Left_Sec_Round_Cooperate");
                else if (startPose == AutoStart.RIGHT)
                    Sec = new PathPlannerAuto("Right_Sec_Round_Cooperate");
                break;
            case No_Cooperate:
                if (startPose == AutoStart.LEFT)
                    Sec = new PathPlannerAuto("Left_Sec_Round");
                else if (startPose == AutoStart.RIGHT)
                    Sec = new PathPlannerAuto("Right_Sec_Round");
                break;
            default:
                break;
        }
        Command End = Commands.none();
        switch (startPose) {
            case LEFT:
                End = new PathPlannerAuto("Left_End");
                break;
            case RIGHT:
                End = new PathPlannerAuto("Right_End");
                break;
            default:
                break;
        }
        return Commands.sequence(start, Sec, End);
    }
}