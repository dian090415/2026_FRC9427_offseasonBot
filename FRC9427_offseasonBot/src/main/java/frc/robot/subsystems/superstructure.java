package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoAilgn;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper.HopperSubsystems;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.Shootersubsystem;
import frc.robot.util.FIeldHelper.AllianceFlipUtil;
import frc.robot.util.OptimizedSteering.DynamicPivotCalculator;
import frc.robot.util.RobotEvent.Event.ShootingStateFalse;
import frc.robot.util.RobotEvent.Event.ShootingStateTrue;
import frc.robot.util.RobotStatus.RobotStatus;
import frc.robot.util.ShooterCalculator.ShooterCalculator;
import frc.robot.util.ShooterCalculator.ShooterCalculator.ShootingState;

public class superstructure extends SubsystemBase {

    private final CommandSwerveDrivetrain drive;
    private final CommandXboxController joystick;
    private final ShooterCalculator shooterCalculator;
    private final RobotStatus robotStatus;
    private final IntakeSubsystem intake;
    private final Shootersubsystem shooter;
    private final HopperSubsystems hopper;
    PIDController thetaController = new PIDController(10.0, 0.0, 0.0);
    private final com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private AngularVelocity flywheelgoal;
    private Angle HoodtargetAngle = Degrees.of(1);

    public superstructure(
            CommandSwerveDrivetrain drive,
            CommandXboxController joystick,
            ShooterCalculator shooterCalculator,
            IntakeSubsystem intake,
            HopperSubsystems hopper,
            Shootersubsystem shooter,
            RobotStatus robotStatus) {
        this.drive = drive;
        this.joystick = joystick;
        this.shooterCalculator = shooterCalculator;
        this.robotStatus = robotStatus;
        this.intake = intake;
        this.shooter = shooter;
        this.hopper = hopper;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ShootingState shooterTargetChoose() {
        if (robotStatus.getArea() == RobotStatus.Area.CENTER) {
            return this.shooterCalculator.calculateShootingToAlliance();
        } else {
            return this.shooterCalculator.calculateShootingToHub();
        }
    }

    public Command intakedown() {
        return Commands.parallel(this.intake.intakerun(), this.hopper.deliver());
    }

    public Command shoottest() {
        return Commands.parallel(this.shooter.shoot(), this.hopper.deliver(), this.intake.intakeupCommand());
    }

    public Command shootstop() {
        return Commands.parallel(this.shooter.stop(), this.hopper.setrollstop(),
                this.intake.intakedownandstop());
    }

    public Command shoot() {
        return Commands.sequence(

                // ✨ 階段 1 & 2 完美合併 ✨
                Commands.parallel(
                        // 持續將最新數據餵給馬達
                        Commands.run(() -> {
                            this.shooter.flywheelsetRPS(flywheelgoal);
                            this.shooter.hoodsetAngle(HoodtargetAngle);
                            this.hopper.setrollshootRPS(RotationsPerSecond.of(60));
                        }),

                        // 底盤持續瞄準
                        new AutoAilgn(this.drive, this)

                ).until(() -> this.shooter.isAtSetPosition() && this.swerveatset()),

                // 階段 3：條件達成！發射並鎖死底盤
                Commands.parallel(
                        Commands.runOnce(() -> this.shooter.tiggerrun()),
                        Commands.runOnce(() -> drive.setX(), this),
                        this.intake.intakeUpAndShootCommand() // 👈 把括號修正，讓它安穩地待在 parallel 裡面
                ) // 👈 在這裡關閉 parallel
        ); // 👈 最後在這裡關閉 sequence
    }

        public Command trenchshoot() {
        return Commands.sequence(

                // ✨ 階段 1 & 2 完美合併 ✨
                Commands.parallel(
                        // 持續將最新數據餵給馬達
                        Commands.run(() -> {
                            this.shooter.flywheelsetRPS(RotationsPerSecond.of(65.27311071747717));
                            this.shooter.hoodsetAngle(Degree.of(13.061510813135357));
                            this.hopper.setrollshootRPS(RotationsPerSecond.of(60));
                        })


                ).until(() -> this.shooter.isAtSetPosition()),

                // 階段 3：條件達成！發射並鎖死底盤
                Commands.parallel(
                        Commands.runOnce(() -> this.shooter.tiggerrun()),
                        Commands.runOnce(() -> drive.setX(), this),
                        this.intake.intakeUpAndShootCommand() // 👈 把括號修正，讓它安穩地待在 parallel 裡面
                ) // 👈 在這裡關閉 parallel
        ); // 👈 最後在這裡關閉 sequence
    }
        public Command townshoot() {
        return Commands.sequence(

                // ✨ 階段 1 & 2 完美合併 ✨
                Commands.parallel(
                        // 持續將最新數據餵給馬達
                        Commands.run(() -> {
                            this.shooter.flywheelsetRPS(RotationsPerSecond.of(64.21101386165485));
                            this.shooter.hoodsetAngle(Degree.of(11.94680182865277));
                            this.hopper.setrollshootRPS(RotationsPerSecond.of(60));
                        })


                ).until(() -> this.shooter.isAtSetPosition()),

                // 階段 3：條件達成！發射並鎖死底盤
                Commands.parallel(
                        Commands.runOnce(() -> this.shooter.tiggerrun()),
                        Commands.runOnce(() -> drive.setX(), this),
                        this.intake.intakeUpAndShootCommand() // 👈 把括號修正，讓它安穩地待在 parallel 裡面
                ) // 👈 在這裡關閉 parallel
        ); // 👈 最後在這裡關閉 sequence
    }
    public boolean swerveatset() {
        Rotation2d targetAngle = this.shooterTargetChoose().FieldAngle();
        Rotation2d currentAngle = drive.getPose2d().getRotation();

        // 使用 .minus() 可以自動處理 360 到 0 度的跨界問題，算出最短角度差
        // 然後取絕對值，看看這個誤差是不是小於 1.5 度
        double errorDegrees = Math.abs(currentAngle.minus(targetAngle).getDegrees());

        return errorDegrees < 1.5;
    }

    public void SetShooterGoal() {
        ShootingState state = this.shooterTargetChoose();

        HoodtargetAngle = state.HoopAngle();

        AngularVelocity FlywheelRPS = state.FlywheelRPS();

        flywheelgoal = FlywheelRPS;

        Logger.recordOutput("HoodTarget", HoodtargetAngle);

        Logger.recordOutput("flywheelgoal", flywheelgoal);

        Logger.recordOutput("Pose", drive.getPose2d());

    }

    @Override
    public void periodic() {
        SetShooterGoal();
        this.shooterCalculator.calculateShootingToAlliance();
        this.shooterCalculator.calculateShootingToHub();
    }

    public Command autoshoot() {
        return Commands.sequence(shoot().withTimeout(3.0));
    }

    public Command autointake() {
        return Commands.parallel(this.intake.intakerun());
    }

    public Command intakestop() {
        return Commands.parallel(this.intake.stopintake());
    }
}
