// package frc.robot.subsystems.Shooter.Tigger;

// import static edu.wpi.first.units.Units.Radians;
// import static edu.wpi.first.units.Units.Rotation;
// import static edu.wpi.first.units.Units.RotationsPerSecond;

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

// public class TiggerHardware implements TriggerIO {
//     private final SparkFlex Motor;

//     // 儲存目前的目標速度，用來判斷是否到達目標
//     private AngularVelocity targetVelocity = RotationsPerSecond.of(0);

//     public TiggerHardware() {
//         this.Motor = new SparkFlex(13, MotorType.kBrushless);
//         configure();
//     }

//     public void configure() {
//         var Config = new SparkFlexConfig();

//         // 1. 基礎設定
//         Config
//                 .idleMode(IdleMode.kCoast)
//                 .inverted(false)
//                 .smartCurrentLimit(40);

//         // 計算齒輪比轉換 (馬達轉一圈對應的弧度)
//         double gearRatio = 1.0 / 3.0;
//         // 2. 編碼器設定
//         Config.encoder
//                 .positionConversionFactor(gearRatio)
//                 // 將預設的 RPM (每分鐘圈數) 轉換為 Radians Per Second (每秒弧度)
//                 .velocityConversionFactor(gearRatio / 60.0);

//         // 3. PID 控制設定
//         Config.closedLoop
//                 .p(0.005)
//                 .i(0)
//                 .d(0.0);

//         // 將設定套用到主馬達
//         Motor.configure(Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     @Override
//     public void setRPS(AngularVelocity RPS) {
//         this.targetVelocity = RPS;
//         Motor.getClosedLoopController().setSetpoint(
//                 RPS.in(RotationsPerSecond),
//                 ControlType.kVelocity,
//                 ClosedLoopSlot.kSlot0);
//     }

//     @Override
//     public AngularVelocity getRPS() {
//         // 從編碼器讀取當前速度 (因為設了 velocityConversionFactor，所以出來直接是 Rads/Sec)
//         return RotationsPerSecond.of(Motor.getEncoder().getVelocity());
//     }

//     @Override
//     public boolean isAtSetPosition() {
//         // 飛輪是看「速度」有沒有到位。計算當前速度與目標速度的誤差。
//         // 如果誤差小於 1.0 rad/s (大約 0.16 圈/秒)，就判定為到達目標。
//         double currentVelocityRadPerSec = getRPS().in(RotationsPerSecond);
//         double targetVelocityRadPerSec = targetVelocity.in(RotationsPerSecond);

//         return Math.abs(currentVelocityRadPerSec - targetVelocityRadPerSec) < 1.0;
//     }
//     @Override
//     public void stop(){
//         this.Motor.stopMotor();
//     }
// }