package frc.robot.subsystems.Intake.Roller;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC; // ✨ 尊爵不凡的 Pro 專屬 FOC 控制
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.*;

public class RollerIOHardware implements RollerIO {

    private final TalonFX roller = new TalonFX(41);
    private final TalonFX rollerfollower = new TalonFX(45);

    private final StatusSignal<AngularVelocity> velocitySignal = roller.getVelocity();

    // ✨ Pro 版專屬的 FOC 力矩電流請求
    private final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

    private double targetRPS = 0.0;

    public RollerIOHardware() {
        velocitySignal.setUpdateFrequency(50);

        // 設定副馬達跟隨主馬達 (反向)
        rollerfollower.setControl(new Follower(roller.getDeviceID(), MotorAlignmentValue.Opposed));

        this.configureMotors();
    }

public void configureMotors() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // 電流限制設定
        configs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20);

        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // ✨ PID 與前饋設定
        configs.Slot0.kP = 0.5;  // 注意：有了 kV 之後，kP 不需要太大，否則會震盪
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        
        // ✨ 在速度控制中，kV (前饋) 是靈魂！這個值代表「要達到 1 RPS 需要多少電壓」
        // 建議先給一個基礎值，例如 0.12，實測後再微調
        configs.Slot0.kV = 0.12; 
        configs.Slot0.kS = 0.2;  // 克服靜摩擦力的基礎電壓

        // 🚨 修正：因為使用了 MotionMagicVelocity，必須設定加速度，否則馬達不會轉！
        configs.MotionMagic.MotionMagicAcceleration = 100.0; // RPS/s (每秒可以加速 100 轉)
        configs.MotionMagic.MotionMagicJerk = 0.0; // 0 代表不使用 S-Curve 平滑

        configs.Feedback.SensorToMechanismRatio = 1.0;

        // 寫入主馬達
        roller.getConfigurator().apply(configs);
        
        // 🛡️ 重要防護：副馬達必須單獨寫入 CurrentLimits 與 NeutralMode
        TalonFXConfiguration followerConfigs = new TalonFXConfiguration();
        followerConfigs.CurrentLimits = configs.CurrentLimits; // 完美複製主馬達的保險絲設定
        followerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerfollower.getConfigurator().apply(followerConfigs);
    }

    @Override
    public void setRPS(AngularVelocity RPS) {
        double targetRPS = RPS.in(RotationsPerSecond);
        this.targetRPS = targetRPS;
        
        // 使用 FOC 送出轉速請求
        roller.setControl(m_request.withVelocity(targetRPS));
    }

    @Override
    public AngularVelocity getRPS() {
        velocitySignal.refresh();
        return velocitySignal.getValue();
    }

    @Override
    public boolean isAtSetPosition() {
        velocitySignal.refresh();
        double currentRPS = velocitySignal.getValue().in(RotationsPerSecond);
        double error = Math.abs(targetRPS - currentRPS);

        if (Math.abs(targetRPS) < 0.1) {
             return Math.abs(currentRPS) < 1.0;
        }
        return error <= 3;
    }

    @Override
    public void stop() {
        this.roller.stopMotor();
    }
}