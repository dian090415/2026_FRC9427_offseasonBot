// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAilgn;
import frc.robot.commands.AutoChooser;
import frc.robot.commands.DrivetrainCmd;
import frc.robot.subsystems.superstructure;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper.HopperSubsystems;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.Shootersubsystem;
import frc.robot.util.FMS.Signal;
import frc.robot.util.RobotStatus.RobotStatus;
import frc.robot.util.ShooterCalculator.ShooterCalculator;

import static edu.wpi.first.units.Units.Newton;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Vision.Limelight;

public class RobotContainer {

          private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain();
  private final RobotStatus robotStatus = new RobotStatus(drivetrain);
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final ShooterCalculator shooterCalculator = new ShooterCalculator(drivetrain, robotStatus);
  private final IntakeSubsystem intake = IntakeSubsystem.create();
  private final HopperSubsystems hopper = HopperSubsystems.create();
  private final Shootersubsystem shooter = Shootersubsystem.create();
  private final superstructure superstructure = new superstructure(drivetrain, joystick, shooterCalculator, intake,
      hopper, shooter, robotStatus);
  private final DrivetrainCmd drivetrainCmd = new DrivetrainCmd(drivetrain, joystick, superstructure);
  public final Signal signal = new Signal();
  private final Dashboard dashboard = new Dashboard(signal);
  private final Limelight limelight = new Limelight(drivetrain, "limelight-up");

  private final AutoChooser autoChooser;

  private final Field2d field = new Field2d();

  public RobotContainer() {
    configureBindings();
    this.autoChooser = new AutoChooser(drivetrain, superstructure);
    log();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrainCmd);
    joystick.leftTrigger().whileTrue(this.superstructure.intakedown());
    joystick.rightTrigger().whileTrue(this.superstructure.shoot()).onFalse(this.superstructure.shootstop());
    // joystick.y().whileTrue(this.superstructure.trenchshoot()).onFalse(this.superstructure.shootstop());
    // joystick.b().whileTrue(this.superstructure.townshoot()).onFalse(this.superstructure.shootstop());
  }

  public void log() {
    SmartDashboard.putData("Field", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("auto/path").setPoses(poses);
    });
  }

  public Command getAutonomousCommand() {
    return autoChooser.auto();
  }
}
