// package frc.robot.subsystems.Shooter.Flywheel;

// import static edu.wpi.first.units.Units.Radians;
// import static edu.wpi.first.units.Units.Rotation;
// import static edu.wpi.first.units.Units.RadiansPerSecond;
// import static edu.wpi.first.units.Units.RotationsPerSecond;

// import org.littletonrobotics.junction.Logger;

// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkFlexConfig;

// import edu.wpi.first.units.measure.AngularVelocity;
// import frc.robot.subsystems.Shooter.ShooterConstants;

// public class FlywheelHardware implements FlywheelIO {
//     private final SparkFlex Main_Motor;
//     private final SparkFlex Follow_Motor_Left;
//     private final SparkFlex Follow_Motor_Right;

//     // 儲存目前的目標速度，用來判斷是否到達目標
//     private AngularVelocity targetVelocity = RotationsPerSecond.of(0);

//     public FlywheelHardware() {
//         this.Main_Motor = new SparkFlex(10, MotorType.kBrushless);
//         this.Follow_Motor_Left = new SparkFlex(11, MotorType.kBrushless);
//         this.Follow_Motor_Right = new SparkFlex(12, MotorType.kBrushless);
//         configure();
//     }

//     @SuppressWarnings("removal")
//     public void configure() {
//         var FlywheelConfig = new SparkFlexConfig();

//         // 1. 基礎設定
//         FlywheelConfig
//                 .idleMode(IdleMode.kCoast)
//                 .inverted(true)
//                 .smartCurrentLimit(80);

//         double gearRatio = 1.0 / 1.4;
//         // 2. 編碼器設定
//         FlywheelConfig.encoder
//                 .positionConversionFactor(gearRatio)
//                 .velocityConversionFactor(gearRatio / 60.0);

//         // 3. PID 控制設定
//         FlywheelConfig.closedLoop
//                 .p(0.9)
//                 .i(0)
//                 .d(0.2)
//                 .velocityFF(1.0 / 80.7);

//         // 將設定套用到主馬達
//         Main_Motor.configure(FlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         // 4. 設定跟隨馬達 (Followers)
//         var Left_FollowerConfig = new SparkFlexConfig();
//         Left_FollowerConfig.follow(Main_Motor, false);
//         var Right_FollowerConfig = new SparkFlexConfig();
//         Right_FollowerConfig.follow(Main_Motor, true);
//         // 如果你的左右馬達機械安裝方向跟主馬達剛好顛倒，改成 .follow(Main_Motor, true)

//         Follow_Motor_Left.configure(Left_FollowerConfig, ResetMode.kResetSafeParameters,
//                 PersistMode.kPersistParameters);
//         Follow_Motor_Right.configure(Right_FollowerConfig, ResetMode.kResetSafeParameters,
//                 PersistMode.kPersistParameters);
//     }

//     @Override
//     public void setRPS(AngularVelocity velocity) {
//         targetVelocity = velocity;

//         Main_Motor.getClosedLoopController().setSetpoint(
//                 velocity.in(RotationsPerSecond),
//                 ControlType.kVelocity,
//                 ClosedLoopSlot.kSlot0);
//     }
//     @Override 
//     public void stop(){
//         this.Main_Motor.stopMotor();
//     }

//     @Override
//     public AngularVelocity getRPS() {
//         return RotationsPerSecond.of(Main_Motor.getEncoder().getVelocity());
//     }

//     @Override
//     public boolean isAtSetPosition() {
//         // 現在兩邊都是統一的圈/秒 (RPS) 單位了
//         Logger.recordOutput("Shooter/Target", targetVelocity.in(RotationsPerSecond));
//         Logger.recordOutput("Shooter/Current", getRPS().in(RotationsPerSecond));
//         Logger.recordOutput(
//                 "Shooter/AppliedOutput",
//                 Main_Motor.getAppliedOutput());

//         Logger.recordOutput(
//                 "Shooter/BusVoltage",
//                 Main_Motor.getBusVoltage());
//         double currentVelocityRPS = getRPS().in(RotationsPerSecond);
//         double targetVelocityRPS = targetVelocity.in(RotationsPerSecond);

//         Logger.recordOutput("erro", Math.abs(currentVelocityRPS - targetVelocityRPS));

//         // 🌟 修正：設定合理的容許誤差。例如 40 目標，實際 38.89 就能判定為 true
//         // 容許誤差設為 1.5 圈/秒 (約 90 RPM)
//         return Math.abs(currentVelocityRPS - targetVelocityRPS) < 1.5;
//     }
// }