package frc.robot.subsystems.Intake.Arm;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ArmHardware implements ArmIO {

        private final TalonFX armMotor;
        private final StatusSignal<Angle> armPosition;
        private final MotionMagicVoltage armOutput = new MotionMagicVoltage(Degree.of(0));

        private final SysIdRoutine sysIdRoutine;
        private final VoltageOut voltagRequire = new VoltageOut(0.0);
        private final StatusSignal<Current> statorCurrentSignal;

        // === 機械傳動常數定義 ===
        public static final double kGearRatio = 12.0;       // 行星齒輪箱減速比 (馬達轉 12 圈 = 輸出軸轉 1 圈)
        public static final double kModule_mm = 3.175;      // 齒輪模數 (mm)
        public static final double kPinionTeeth = 24.0;     // 驅動齒盤齒數

        // 1. 計算節圓周長 (公尺)：PCD(mm) = 模數 * 齒數，轉成公尺後乘上 Pi
        public static final double kPitchCircleCircumference_m = (kModule_mm * kPinionTeeth / 1000.0) * Math.PI;

        // 2. 計算馬達每轉一圈 (Rotor Rotation)，齒條前進的公尺數
        public static final double kMetersPerRotorRotation = kPitchCircleCircumference_m / kGearRatio;

        // 3. 最大伸長極限 (依據你原本的 0.3 公尺設定)
        public static final double kMaxExtensionMeters = 0.26;
        public static final double kMaxExtensionRotations = kMaxExtensionMeters / kMetersPerRotorRotation;

        public ArmHardware() {
                this.armMotor = new TalonFX(30);
                this.armPosition = armMotor.getPosition();
                this.statorCurrentSignal = armMotor.getStatorCurrent();
                statorCurrentSignal.setUpdateFrequency(50);
                configure();
                // resetEncoder();

                SignalLogger.setPath("/U/");

                this.sysIdRoutine = new SysIdRoutine(
                                new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(1),
                                                null, (state) -> SignalLogger.writeString("state", state.toString())),
                                new SysIdRoutine.Mechanism(
                                                (volts) -> this.armMotor
                                                                .setControl(voltagRequire.withOutput(volts.in(Volts))),
                                                null,
                                                new SubsystemBase() {
                                                        @Override
                                                        public String getName() {
                                                                return "TurretSysId";
                                                        }
                                                }));
        }

        @Override
        public void setPosition(double m) {
                // 將公尺換算為馬達圈數
                double rot = m / kMetersPerRotorRotation;
                armMotor.setControl(armOutput.withPosition(rot));
        }

        @Override
        public double getPosition() {
                this.armPosition.refresh();
                // 將馬達圈數換算回公尺
                return this.armPosition.getValueAsDouble() * kMetersPerRotorRotation;
        }

        @Override
        public void resetEncoder() {
                this.armMotor.getConfigurator().setPosition(Degree.of(0.0));
        }

        @Override
        public void configure() {
                var IntakeArmConfig = new TalonFXConfiguration();

                IntakeArmConfig.CurrentLimits
                                .withStatorCurrentLimitEnable(true)
                                .withStatorCurrentLimit(40)
                                .withSupplyCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(20);

                IntakeArmConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                IntakeArmConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                IntakeArmConfig.SoftwareLimitSwitch
                                .withReverseSoftLimitEnable(true)
                                .withReverseSoftLimitThreshold(0.0) // 使用正確的圈數極限
                                .withForwardSoftLimitEnable(true)
                                .withForwardSoftLimitThreshold(kMaxExtensionRotations);

                IntakeArmConfig.Feedback.SensorToMechanismRatio = 1.0; // 已經在程式碼中用 kMetersPerRotorRotation 處理了，這裡維持 1.0 避免雙重除算

                IntakeArmConfig.Slot0.kP = 75.0;
                IntakeArmConfig.Slot0.kI = 0.0;
                IntakeArmConfig.Slot0.kD = 0.0;
                IntakeArmConfig.Slot0.kG = 0.0;
                IntakeArmConfig.Slot0.kA = 0.0;
                IntakeArmConfig.Slot0.kS = 0.0;
                IntakeArmConfig.Slot0.kV = 0.0;
                IntakeArmConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

                IntakeArmConfig.MotionMagic
                                .withMotionMagicCruiseVelocity(DegreesPerSecond.of(10800))
                                .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(72000));

                armMotor.getConfigurator().apply(IntakeArmConfig);
        }

        @Override
        public Command sysid() {
                return Commands.sequence(
                                Commands.runOnce(() -> {
                                        SignalLogger.start();
                                        armMotor.getPosition().setUpdateFrequency(250);
                                        armMotor.getVelocity().setUpdateFrequency(250);
                                        armMotor.getMotorVoltage().setUpdateFrequency(250);
                                }),

                                // ⚠️ 注意：因為 getPosition() 現在回傳的是公尺，所以這裡抓取原始圈數來維持你原本的 55 與 135 邏輯
                                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                                                .until(() -> {
                                                        armPosition.refresh();
                                                        return armPosition.getValueAsDouble() < 55;
                                                }),
                                new WaitCommand(1.5),

                                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                                                .until(() -> {
                                                        armPosition.refresh();
                                                        return armPosition.getValueAsDouble() > 135;
                                                }),
                                new WaitCommand(1.5),

                                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                                                .until(() -> {
                                                        armPosition.refresh();
                                                        return armPosition.getValueAsDouble() < 55;
                                                }),
                                new WaitCommand(1.5),

                                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                                                .until(() -> {
                                                        armPosition.refresh();
                                                        return armPosition.getValueAsDouble() > 135;
                                                }),

                                Commands.runOnce(() -> {
                                        System.err.println("🛑 SysId 紀錄結束！");
                                        SignalLogger.stop();
                                        armMotor.getPosition().setUpdateFrequency(50);
                                        armMotor.getVelocity().setUpdateFrequency(50);
                                        armMotor.getMotorVoltage().setUpdateFrequency(50);
                                }));
        }

        @Override
        public boolean havefuel() {
                statorCurrentSignal.refresh();
                double loadCurrent = statorCurrentSignal.getValue().in(Amps);
                Logger.recordOutput("armloadCurrent", loadCurrent);
                return false;
        }
}