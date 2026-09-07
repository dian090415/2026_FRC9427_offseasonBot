package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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

public class AutoAilgn extends Command {
    private final CommandSwerveDrivetrain drive;
    private final superstructure superSystem;
    private final PIDController thetaController;;

    // 建構子 (Constructor)
    public AutoAilgn(CommandSwerveDrivetrain drive, superstructure superSystem) {
        this.drive = drive;
        this.superSystem = superSystem;

        // 宣告轉向專用的 PID，並開啟連續輸入以防止 180 度死亡抽搐
        this.thetaController = new PIDController(6.0, 0.0, 0.0);
        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.thetaController.setTolerance(Math.toRadians(3.0));

        // 宣告這個 Command 需要佔用底盤子系統
        addRequirements(drive);
    }

    @Override
    public void execute() {

        double rotSpeed = 0.0;
        Rotation2d targetAngle = superSystem.shooterTargetChoose().FieldAngle();
        Rotation2d currentAngle = drive.getPose2d().getRotation();
        rotSpeed = thetaController.calculate(currentAngle.getRadians(), targetAngle.getRadians());
        rotSpeed = MathUtil.clamp(rotSpeed,
                -DriveConstants.kMaxAngularSpeedRadiansPerSecond,
                DriveConstants.kMaxAngularSpeedRadiansPerSecond);
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(0.0, 0.0, rotSpeed);

        // ✨ 物理邏輯修正：轉換為機器人相對速度，並套用動態圓心
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, currentAngle);
        Translation2d bestPivot = DynamicPivotCalculator.getBestPivotWheel(robotSpeeds);
        ChassisSpeeds compensatedSpeeds = DynamicPivotCalculator.calculateOffsetCompensation(robotSpeeds,
                bestPivot);

        drive.aimAroundPivot(compensatedSpeeds, bestPivot);
    }
}