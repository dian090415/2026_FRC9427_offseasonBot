package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.superstructure;
import frc.robot.util.FIeldHelper.AllianceFlipUtil;
import frc.robot.util.OptimizedSteering.DynamicPivotCalculator; // 記得引入動態圓心計算器

public class DrivetrainCmd extends Command {
    private final CommandSwerveDrivetrain drive;
    private final CommandXboxController joystick;
    private final superstructure superSystem;
    private final PIDController thetaController;
    private final double slow_down = 0.3;

    // 建構子 (Constructor)
    public DrivetrainCmd(CommandSwerveDrivetrain drive, CommandXboxController joystick, superstructure superSystem) {
        this.drive = drive;
        this.joystick = joystick;
        this.superSystem = superSystem;

        // 宣告轉向專用的 PID，並開啟連續輸入以防止 180 度死亡抽搐
        this.thetaController = new PIDController(10.0, 0.0, 0.0);
        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 宣告這個 Command 需要佔用底盤子系統
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // 1. 讀取原始搖桿輸入
        double xSpeed = MathUtil.applyDeadband(-joystick.getLeftY(), 0.05) * DriveConstants.kMaxSpeedMeterPerSecond;
        double ySpeed = MathUtil.applyDeadband(-joystick.getLeftX(), 0.05) * DriveConstants.kMaxSpeedMeterPerSecond;

        // ✨ 2. 【關鍵修復】紅藍方視角反轉
        // WPILib 的 +X 永遠朝向紅方。如果你是紅方司機，搖桿往前推應該是要「遠離自己 (朝向藍方 = -X)」
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
            xSpeed = -xSpeed;
            ySpeed = -ySpeed;
        }

        double rotSpeed = 0.0;

        if (joystick.getHID().getRightBumper()) {

            // 取得目標角度與當前角度
            Rotation2d targetAngle = superSystem.shooterTargetChoose().FieldAngle();
            Rotation2d currentAngle = drive.getPose2d().getRotation();

            // PID 計算旋轉速度並限制最大值
            rotSpeed = thetaController.calculate(currentAngle.getRadians(), targetAngle.getRadians());
            rotSpeed = MathUtil.clamp(rotSpeed,
                    -DriveConstants.kMaxAngularSpeedRadiansPerSecond,
                    DriveConstants.kMaxAngularSpeedRadiansPerSecond);

            // 建立場地相對速度
            ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xSpeed * slow_down, ySpeed * slow_down, rotSpeed);

            // ✨ 物理邏輯修正：轉換為機器人相對速度，並套用動態圓心
            ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, currentAngle);
            Translation2d bestPivot = DynamicPivotCalculator.getBestPivotWheel(robotSpeeds);
            ChassisSpeeds compensatedSpeeds = DynamicPivotCalculator.calculateOffsetCompensation(robotSpeeds,
                    bestPivot);

            drive.aimAroundPivot(compensatedSpeeds, bestPivot);

        } else {
            // 手動駕駛模式
            rotSpeed = MathUtil.applyDeadband(-joystick.getRightX() * 0.8, 0.05)
                    * DriveConstants.kMaxAngularSpeedRadiansPerSecond;

            ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
            drive.runVelocity(fieldSpeeds);
        }
    }
}