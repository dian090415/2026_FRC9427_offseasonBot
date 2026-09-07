// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Swerve.ModuleLimits;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public static class OperatorConstants {
                public static final int kDriverControllerPort = 0;
        }

        public static final class SwerveModuleConstants {
                public static final String[] ModuleName = {
                                "ForntLeft",
                                "FrontRight",
                                "BackLeft",
                                "BackRight"
                };
        }

        public static final class FieldConstants {
                public static final AprilTagFieldLayout layout = AprilTagFieldLayout
                                .loadField(AprilTagFields.kDefaultField);
                public static final double fieldLength;
                public static final double fieldWidth;
                static {
                        AprilTagFieldLayout layout;
                        try {
                                // 自動載入當年度的預設場地 (例如 2026 場地)
                                layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                        } catch (Exception e) {
                                // 萬一讀不到檔案 (極少發生)，給個預設值防止程式崩潰
                                // 這裡可以填入規則書上的大約數值
                                layout = null;
                                e.printStackTrace();
                        }

                        if (layout != null) {
                                // 從官方資料直接抓取精確數值
                                fieldLength = layout.getFieldLength();
                                fieldWidth = layout.getFieldWidth();
                        } else {
                                // Fallback (保底數值)
                                fieldLength = 16.54;
                                fieldWidth = 8.21;
                        }
                }

                public class siteConstants {
                        // Dimensions
                        public static final double width = Units.inchesToMeters(31.8);
                        public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
                        public static final double height = Units.inchesToMeters(7.0);
                        public static final double bumpers = Units.inchesToMeters(73.0);
                        public static final double hub = Units.inchesToMeters(47.0);

                        public static final double TRENCHWide = Units.inchesToMeters(65.65);
                        public static final double TRENCH = Units.inchesToMeters(6);
                        public static final double TRENCHdeep = Units.inchesToMeters(47.0);
                        public static final double HUB_distance_to_the_ALLIANCE_WALL = Units.inchesToMeters(158.6);

                        public static final Translation3d topCenterPoint = new Translation3d(
                                        4.022 + width / 2.0,
                                        fieldWidth / 2.0, // Y 軸置中
                                        height // 高度固定
                        );
                        public static final Translation3d topLeftCenterPoint = new Translation3d(
                                        4.022 + width / 2.0,
                                        (fieldWidth / 2.0) + (bumpers / 2 + hub / 2), // Y 軸置中
                                        height // 高度固定
                        );
                        public static final Translation3d topRightCenterPoint = new Translation3d(
                                        4.022 + width / 2.0,
                                        (fieldWidth / 2.0) - (bumpers / 2 + hub / 2), // Y 軸置中
                                        height // 高度固定
                        );

                        public static final Pose2d Right_TRENCHE_Pose1 = new Pose2d(
                                        HUB_distance_to_the_ALLIANCE_WALL + (TRENCHWide / 2 - TRENCH / 2), 0.0,
                                        new Rotation2d(0.0));
                        public static final Pose2d Right_TRENCHE_Pose2 = new Pose2d(
                                        HUB_distance_to_the_ALLIANCE_WALL + (TRENCHWide / 2 - TRENCH / 2) + TRENCH,
                                        0.0, new Rotation2d(0.0));
                        public static final Pose2d Right_TRENCHE_Pose3 = new Pose2d(
                                        HUB_distance_to_the_ALLIANCE_WALL + (TRENCHWide / 2 - TRENCH / 2), TRENCHdeep,
                                        new Rotation2d(0.0));
                        public static final Pose2d Right_TRENCHE_Pose4 = new Pose2d(
                                        HUB_distance_to_the_ALLIANCE_WALL + (TRENCHWide / 2 - TRENCH / 2) + TRENCH,
                                        TRENCHdeep,
                                        new Rotation2d(0.0));

                        public static final Pose2d Left_TRENCHE_Pose1 = new Pose2d(
                                        HUB_distance_to_the_ALLIANCE_WALL + (TRENCHWide / 2 - TRENCH / 2),
                                        FieldConstants.fieldWidth, new Rotation2d(0.0));
                        public static final Pose2d Left_TRENCHE_Pose2 = new Pose2d(
                                        HUB_distance_to_the_ALLIANCE_WALL + (TRENCHWide / 2 - TRENCH / 2) + TRENCH,
                                        FieldConstants.fieldWidth, new Rotation2d(0.0));
                        public static final Pose2d Left_TRENCHE_Pose3 = new Pose2d(
                                        HUB_distance_to_the_ALLIANCE_WALL + (TRENCHWide / 2 - TRENCH / 2),
                                        FieldConstants.fieldWidth - TRENCHdeep,
                                        new Rotation2d(0.0));
                        public static final Pose2d Left_TRENCHE_Pose4 = new Pose2d(
                                        HUB_distance_to_the_ALLIANCE_WALL + (TRENCHWide / 2 - TRENCH / 2) + TRENCH,
                                        FieldConstants.fieldWidth - TRENCHdeep,
                                        new Rotation2d(0.0));
                }
        }

        public static final class DriveConstants {

                public static final Translation2d[] autoLocations = new Translation2d[] {
                                new Translation2d(0.355, 0.330),
                                new Translation2d(0.355, -0.330),
                                new Translation2d(-0.355, 0.330),
                                new Translation2d(-0.355, -0.330)
                };
                public static final Translation2d[] moduleLocations = new Translation2d[] {
                                new Translation2d(0.355, 0.330),
                                new Translation2d(0.355, -0.330),
                                new Translation2d(-0.355, 0.330),
                                new Translation2d(-0.355, -0.330)
                };

                public static final double kMaxSpeedMeterPerSecond = 6.0;
                public static final double kMaxAngularSpeedRadiansPerSecond = 3 * 2 * Math.PI;

                public static final double kMaxAccerationUnitsPerSecond = 22;

                public static final ModuleLimits moduleLimitsFree = new ModuleLimits(kMaxSpeedMeterPerSecond,
                                kMaxAccerationUnitsPerSecond, Units.degreesToRadians(1800));
        }

        public static final class PhotonVisionConstants {

                public static final Map<String, Transform3d> cameraTransforms = Map.of(
                                "FrontRight", new Transform3d(
                                                // 右側
                                                new Translation3d(0.3256159, -0.0494339, 0.2242014),
                                                new Rotation3d(0.0, Units.degreesToRadians(-30 + .0),
                                                                Units.degreesToRadians(30))),
                                "FrontLeft", new Transform3d(
                                                // 左側
                                                new Translation3d(0.3256159, 0.0494339, 0.2242014),
                                                new Rotation3d(0.0, Units.degreesToRadians(-30.0),
                                                                Units.degreesToRadians(-30))));

                public static final double borderPixels = 15.0; // 拒絕貼邊緣的角點（避免畸變/遮擋）
                public static final double maxSingleTagDistanceMeters = Units.feetToMeters(10); // 單tag最遠可接受距離
                public static final double maxYawRate = 720.0;// 最大可以接受的旋轉速度
                public static final double maxZ = 0.5; // 最大高度
        }

        public final class SwerveConstants {

                public static final double kTrackWidth = edu.wpi.first.math.util.Units.inchesToMeters(22.75);// 左到右 距離
                public static final double kWheelBase = edu.wpi.first.math.util.Units.inchesToMeters(20.75);// 前到後 距輪
                public static final double kDriveGearRatio = 4.71; // L3 驅動齒比
                public static final double kSteerGearRatio = 287.0 / 11.0;// 轉向齒比
                public static final double kMaxSpeed = 4;// 最大速度
                public static final double turnSpeed = 1;
                public static final InvertedValue kFLDriveInverted = InvertedValue.CounterClockwise_Positive;
                public static final InvertedValue kBLDriveInverted = InvertedValue.CounterClockwise_Positive;
                public static final InvertedValue kFRDriveInverted = InvertedValue.Clockwise_Positive;
                public static final InvertedValue kBRDriveInverted = InvertedValue.Clockwise_Positive;

                // Swerve 運動學 (SwerveDriveKinematics)
                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // FL
                                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // FR
                                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // BL
                                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) // BR
                );

                // 右前輪 (FR)
                public static final int kFRDriveId = 2;
                public static final int kFRSteerId = 46;
                public static final int kFREncoderId = 1;
                public static final double kFROffset = 0.289794921875;
                // 右後輪 (BR)
                public static final int kBRDriveId = 4;
                public static final int kBRSteerId = 3;
                public static final int kBREncoderId = 2;
                public static final double kBROffset = 0.19580078125;
                // 左後輪 (BL)
                public static final int kBLDriveId = 8;
                public static final int kBLSteerId = 7;
                public static final int kBLEncoderId = 4;
                public static final double kBLOffset = 0.30126953125;
                // 左前輪 (FL)
                public static final int kFLDriveId = 6;
                public static final int kFLSteerId = 5;
                public static final int kFLEncoderId = 3;
                public static final double kFLOffset = -0.258544921875;
                // 陀螺儀 (Pigeon)
                public static final int kPigeonId = 11;

                // 轉向 (Steer) PID 控制參數
                public static final double kSteerkP = 11.0000 * (2 * Math.PI);
                public static final double kSteerkI = 0.0000 * (2 * Math.PI);
                public static final double kSteerkD = 0.0000 * (2 * Math.PI);
                public static final double kSteerSupplyCurrentLimit = 40.0;// 電流限制
                public static final boolean kSteerSupplyCurrentLimitEnable = true;// 打開電流限制
                // 驅動 (Drive) PID 與前饋 (Feedforward) 控制參數
                public static final double kWheelRadius = edu.wpi.first.math.util.Units.inchesToMeters(2.0);// 輪子半徑2inch
                public static final double kWheelCircumference = 2 * Math.PI * kWheelRadius;// 輪子周長
                public static final double kDrivekP = 0.0021 * kWheelCircumference;
                public static final double kDrivekI = 0.0;
                public static final double kDrivekD = 0.0;
                public static final double kDrivekS = 0.18; // 克服靜摩擦力
                public static final double kDrivekV = 2.35; // 速度常數
                public static final double kDrivekA = 0.05; // 加速度常數

                public static final double kDriveSupplyCurrentLimit = 55.0;// 電流限制
                public static final boolean kDriveSupplyCurrentLimitEnable = true;// 打開電流限制
                public static final double ksteerS = 40;

        }

        public static final class LimelightConstants {
                public static final double MAX_GYRO_RATE = 1080;
        }
}
