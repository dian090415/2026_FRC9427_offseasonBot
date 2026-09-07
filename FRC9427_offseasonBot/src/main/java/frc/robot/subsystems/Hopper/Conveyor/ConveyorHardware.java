// package frc.robot.subsystems.Hopper.Conveyor;

// import static edu.wpi.first.units.Units.RotationsPerSecond;

// import com.ctre.phoenix6.StatusSignal;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkFlexConfig;

// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Current;

// public class ConveyorHardware implements ConveyorIO {

//     private final SparkFlex motor;
//     private final SparkClosedLoopController controller;

//     private AngularVelocity targetVelocity = RotationsPerSecond.of(0);

//             private final StatusSignal<Current> statorCurrentSignal;

//     public ConveyorHardware() {
//         this.motor = new SparkFlex(46, MotorType.kBrushless);
//         this.controller = motor.getClosedLoopController();
//         this.statorCurrentSignal = armMotor.getStatorCurrent();

//         configure();
//         // 輸送帶 (Conveyor) 是看速度的，通常不需要歸零位置 (resetPosition)
//     }

//     @SuppressWarnings("removal")
//     public void configure() {
//         var config = new SparkFlexConfig();

//         // 1. 設定 Motor (保留你新設定的 Brake 煞車模式與反轉)
//         config
//                 .idleMode(IdleMode.kBrake)
//                 .inverted(true)
//                 .smartCurrentLimit(40);

//         // 2. 編碼器轉換設定 (套用回上一版的 1:3 行星齒輪比設定)
//         double gearRatio = 1.0 / 3.0;
//         config.encoder
//                 .positionConversionFactor(gearRatio)
//                 .velocityConversionFactor(gearRatio / 60.0);

//         // 3. 將 PID 掛載到馬達 (套用回上一次的保守 P 值，避免爆衝)
//         config.closedLoop
//                 .pid(0.005, 0.0, 0.0);

//         // 4. 一次性安全寫入馬達配置
//         // 🟢 依照你的註解，這裡已修正為 kNoResetSafeParameters
//         this.motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     // --- 以下為 ConveyorIO 必須實作的介面方法 (幫你加回來) ---

//     @Override
//     public void setRPS(AngularVelocity rps) {
//         this.targetVelocity = rps;
//         controller.setSetpoint(
//                 rps.in(RotationsPerSecond),
//                 ControlType.kVelocity);
//     }

//     @Override
//     public AngularVelocity getRPS() {
//         return RotationsPerSecond.of(motor.getEncoder().getVelocity());
//     }
//     @Override
//     public void stop(){
//         this.motor.stopMotor();
//     }

//     @Override
//     public boolean isAtSetPosition() {
//         return Math.abs(
//                 getRPS().in(RotationsPerSecond)
//                         - targetVelocity.in(RotationsPerSecond)) < 0.16;
//     }
// }