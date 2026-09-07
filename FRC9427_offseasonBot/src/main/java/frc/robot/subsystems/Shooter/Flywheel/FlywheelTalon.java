package frc.robot.subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import javax.tools.Diagnostic;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class FlywheelTalon implements FlywheelIO {
    private final TalonFX flywheel = new TalonFX(9);
    private final TalonFX Follow_Motor_Left = new TalonFX(10);
    private final TalonFX Follow_Motor_Right = new TalonFX(11);
    private final TalonFX Follow_Motor = new TalonFX(12);

    private final StatusSignal<AngularVelocity> velocitySignal = flywheel.getVelocity();

    private final StatusSignal<Current> statorCurrentSignal;

    private final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

    private double targetRPS = 0.0;

    public FlywheelTalon() {
        velocitySignal.setUpdateFrequency(50);
        this.statorCurrentSignal = flywheel.getStatorCurrent();
        statorCurrentSignal.setUpdateFrequency(50);

        Follow_Motor_Left.setControl(new Follower(flywheel.getDeviceID(), MotorAlignmentValue.Aligned));
        Follow_Motor_Right.setControl(new Follower(flywheel.getDeviceID(), MotorAlignmentValue.Opposed));
        Follow_Motor.setControl(new Follower(flywheel.getDeviceID(), MotorAlignmentValue.Opposed));

        this.configureMotors();
    }

    public void configureMotors() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // 修正 3: 電流限制寫法精簡化 (你原本寫了兩次 SupplyLimit)
        // Stator (定子電流): 限制加速時的爆發力 -> 設 60-80A 防止燒馬達
        // Supply (供應電流): 限制電池端的耗電 -> 設 40A 防止像上次那樣電壓驟降
        configs.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(35)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20); // 60 -> 40

        // 馬達設定
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Roller 通常用 Coast，停下來比較滑順
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // 修正 4: PID 參數 (Torque 模式專用)
        // 這些數值需要重新用 SysId 測量，或者手動調整
        // 這裡給的是 "經驗法則" 的預估值，比你原本的大很多

        configs.Slot0.kP = 9.5; // 誤差 1 RPS，給 5 安培修正 (比 0.11 有力多了)
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;

        // Torque 模式下，kV 通常很小或為 0 (TalonFX 內部會處理反電動勢)
        // 這裡的 kV 只是用來對抗空氣阻力和摩擦力
        configs.Slot0.kV = 0.0;

        // kS (靜摩擦): 要給多少安培才推得動？
        configs.Slot0.kS = 0.0;

        // 修正 5: 移除 MotionMagic 設定
        // 因為我們現在是用 Velocity 模式，不需要設定 CruiseVelocity 和 Acceleration
        // 如果你希望加速不要太快，可以用 configs.ClosedLoopRamps.TorqueClosedLoopRampPeriod

        configs.Feedback.SensorToMechanismRatio = 1.0;

        configs.MotionMagic.MotionMagicAcceleration = 100.0; // RPS/s (每秒可以加速 100 轉)
        configs.MotionMagic.MotionMagicJerk = 0.0; // 0 代表不使用 S-Curve 平滑

        flywheel.getConfigurator().apply(configs);
    }

    @Override
    public void setRPS(AngularVelocity RPS) {
        double targetRPS = RPS.in(RotationsPerSecond);

        this.targetRPS = targetRPS;

        flywheel.setControl(m_request.withVelocity(targetRPS));
    }

    @Override
    public AngularVelocity getRPS() {
        velocitySignal.refresh();

        return velocitySignal.getValue();
    }

    @Override
    public boolean isAtSetPosition() {
        // 1. 刷新數據
        velocitySignal.refresh();

        // 2. 取得目前實際轉速
        double currentRPS = velocitySignal.getValue().in(RotationsPerSecond);

        // 3. 計算誤差絕對值
        double error = Math.abs(targetRPS - currentRPS);

        Logger.recordOutput("currentRPS", currentRPS);
        Logger.recordOutput("targetRPS", targetRPS);
        Logger.recordOutput("error", error);

        if (Math.abs(targetRPS) < 0.1) {
            return Math.abs(currentRPS) < 1.0;
        }

        return error <= 3;
    }

    @Override
    public void stop() {
        this.flywheel.stopMotor();
    }

    @Override
    public boolean havefuel() {
        statorCurrentSignal.refresh();
        double loadCurrent = statorCurrentSignal.getValue().in(Amps);
        Logger.recordOutput("flywheelloadCurrent", loadCurrent);
        return false;
    }
}
