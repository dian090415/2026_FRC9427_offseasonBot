/*
 * Original code from Littleton Robotics (Team 6328) - 2026 Season
 * Modified by Team [10114]
 * * Licensed under the MIT License.
 */

package frc.robot.util.ShooterCalculator;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.FieldConstants.siteConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.util.FIeldHelper.AllianceFlipUtil;
import frc.robot.util.RobotStatus.RobotStatus;

public class ShooterCalculator {
        private final RobotStatus robotStatus;
        private final CommandSwerveDrivetrain drive;
        private final InterpolatingTreeMap<Double, Angle> hoodMap;
        private final InterpolatingTreeMap<Double, AngularVelocity> rollMap;
        private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
        private final InterpolatingTreeMap<Double, AngularVelocity> ToAillancerollMap;
        private static final InterpolatingDoubleTreeMap ToAillancetimeOfFlightMap = new InterpolatingDoubleTreeMap();
        private final double time_error = 0.0;
        private final double phaseDelay = 0.03 + time_error;
        private final double linearDragTimeConstant = 0.375;

        private static final Angle Hood_MAX_RADS = ShooterConstants.Hood_MAX_LIMIT;

        public ShooterCalculator(CommandSwerveDrivetrain drive, RobotStatus robotStatus) {
                this.drive = drive;
                this.robotStatus = robotStatus;

                hoodMap = new InterpolatingTreeMap<>(
                                InverseInterpolator.forDouble(),
                                (start, end, t) -> {
                                        // 1. 把單位轉成 double (用 Radians 或 Degrees 都可以，統一就好)
                                        double startVal = start.in(Degree);
                                        double endVal = end.in(Degree);
                                        // 2. 算數學插值 (start + (end - start) * t)
                                        double result = MathUtil.interpolate(startVal, endVal, t);

                                        // 3. 把 double 包回 Angle 物件
                                        return Degree.of(result);
                                });
                rollMap = new InterpolatingTreeMap<>(
                                InverseInterpolator.forDouble(),
                                (start, end, t) -> {
                                        // 邏輯：拆成 double (RPM) -> 算數學 -> 包回 Unit
                                        double startVal = start.in(RotationsPerSecond);
                                        double endVal = end.in(RotationsPerSecond);
                                        double interpolated = MathUtil.interpolate(startVal, endVal, t);
                                        return RotationsPerSecond.of(interpolated);
                                });
                ToAillancerollMap = new InterpolatingTreeMap<>(
                                InverseInterpolator.forDouble(),
                                (start, end, t) -> {
                                        // 邏輯：拆成 double (RPM) -> 算數學 -> 包回 Unit
                                        double startVal = start.in(RotationsPerSecond);
                                        double endVal = end.in(RotationsPerSecond);
                                        double interpolated = MathUtil.interpolate(startVal, endVal, t);
                                        return RotationsPerSecond.of(interpolated);
                                });

                rollMap.put(2.017127, RotationsPerSecond.of(36));
                rollMap.put(2.46169, RotationsPerSecond.of(40.5));
                rollMap.put(3.102784, RotationsPerSecond.of(42.0));
                rollMap.put(3.102784, RotationsPerSecond.of(45.0));

                hoodMap.put(2.017127, Degree.of(3.0));
                hoodMap.put(2.46169, Degree.of(8.0));
                hoodMap.put(3.102784, Degree.of(10.0));
                hoodMap.put(4.071786, Degree.of(20.0));

                timeOfFlightMap.put(1.661037 - 0.370866, 0.84);
                timeOfFlightMap.put(2.429462 - 0.370866, 0.82);
                timeOfFlightMap.put(2.918019 - 0.370866, 0.93);
                timeOfFlightMap.put(3.624158 - 0.370866, 0.93);
                timeOfFlightMap.put(4.08147 - 0.370866, 0.97);
                timeOfFlightMap.put(4.446884 - 0.370866, 1.0);

                ToAillancerollMap.put(1.661037 - 0.370866, RotationsPerSecond.of(49.8));
                ToAillancerollMap.put(2.429462 - 0.370866, RotationsPerSecond.of(53.5));
                ToAillancerollMap.put(2.918019 - 0.370866, RotationsPerSecond.of(58.0));
                ToAillancerollMap.put(3.624158 - 0.370866, RotationsPerSecond.of(62.5));
                ToAillancerollMap.put(4.08147 - 0.370866, RotationsPerSecond.of(63.7));
                ToAillancerollMap.put(4.446884 - 0.370866, RotationsPerSecond.of(66.0));

                ToAillancetimeOfFlightMap.put(1.661037 - 0.370866, 0.84);
                ToAillancetimeOfFlightMap.put(2.429462 - 0.370866, 0.82);
                ToAillancetimeOfFlightMap.put(2.918019 - 0.370866, 0.93);
                ToAillancetimeOfFlightMap.put(3.624158 - 0.370866, 0.93);
                ToAillancetimeOfFlightMap.put(4.08147 - 0.370866, 0.97);
                ToAillancetimeOfFlightMap.put(4.446884 - 0.370866, 1.0);

        }

        public record ShootingState(
                        Rotation2d FieldAngle,
                        Angle HoopAngle,
                        AngularVelocity FlywheelRPS) {
        }

        public ShootingState calculateShootingToHub() {
                Pose2d estimatedPose = drive.getPose2d();
                ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

                // 1. 延遲補償
                estimatedPose = estimatedPose.exp(new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

                Pose2d turretPosition = estimatedPose.transformBy(
                                new Transform2d(
                                                ShooterConstants.robotToLLauch.getTranslation().toTranslation2d(),
                                                ShooterConstants.robotToLLauch.getRotation().toRotation2d()));

                Translation2d target = AllianceFlipUtil.apply(siteConstants.topCenterPoint.toTranslation2d());
                double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

                // 2. 砲塔場地速度計算 (✨ 修正了外積的正負號問題)
                ChassisSpeeds robotVelocity = drive.getFieldVelocity();
                if (AllianceFlipUtil.shouldFlip()) {
                        robotVelocity = new ChassisSpeeds(
                                        -robotVelocity.vxMetersPerSecond,
                                        -robotVelocity.vyMetersPerSecond,
                                        robotVelocity.omegaRadiansPerSecond);
                }

                Translation2d turretOffsetField = ShooterConstants.robotToLLauch.getTranslation().toTranslation2d()
                                .rotateBy(estimatedPose.getRotation());

                double turretVelocityX = robotVelocity.vxMetersPerSecond
                                - (robotVelocity.omegaRadiansPerSecond * turretOffsetField.getY());
                double turretVelocityY = robotVelocity.vyMetersPerSecond
                                + (robotVelocity.omegaRadiansPerSecond * turretOffsetField.getX());

                // 3. 迭代預測虛擬目標點 (Lookahead Pose)
                double timeOfFlight = 0.0;
                Pose2d lookaheadPose = turretPosition;
                double lookaheadTurretToTargetDistance = turretToTargetDistance;

                for (int i = 0; i < 5; i++) {
                        timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);

                        // 升級：空氣阻力補償 (Effective TOF)
                        // 避免高速平移時預測過頭，算出被空氣阻力衰減後的「有效飛行時間」
                        double effectiveTOF = (1 - Math.exp(-timeOfFlight * linearDragTimeConstant))
                                        / linearDragTimeConstant;

                        double offsetX = turretVelocityX * effectiveTOF;
                        double offsetY = turretVelocityY * effectiveTOF;

                        lookaheadPose = new Pose2d(
                                        turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                                        turretPosition.getRotation());

                        lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
                }
                Logger.recordOutput("lookaheadTurretToTargetDistance", lookaheadTurretToTargetDistance);

                // 4. 計算最終對正角度與查表
                Translation2d vectorToTarget = target.minus(lookaheadPose.getTranslation());
                Rotation2d targetFieldAngle = vectorToTarget.getAngle();

                // Logger.recordOutput("hubdriveFieldAngle",
                // new Pose2d(drive.getPose2d().getX(), drive.getPose2d().getY(),
                // targetFieldAngle));

                return new ShootingState(
                                targetFieldAngle,
                                hoodMap.get(lookaheadTurretToTargetDistance),
                                rollMap.get(lookaheadTurretToTargetDistance));
        }

        // -------------------------------------------------------------------------------------------------------------------

        public ShootingState calculateShootingToAlliance() {
                Pose2d estimatedPose = drive.getPose2d();
                ChassisSpeeds robotRelativeVelocity = drive.getChassisSpeeds();

                // 1. 延遲補償
                estimatedPose = estimatedPose.exp(new Twist2d(
                                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

                Pose2d turretPosition = estimatedPose.transformBy(
                                new Transform2d(
                                                ShooterConstants.robotToLLauch.getTranslation().toTranslation2d(),
                                                ShooterConstants.robotToLLauch.getRotation().toRotation2d()));

                Translation2d target;
                if (robotStatus.getVerticalSide() == RobotStatus.VerticalSide.TOP) {
                        target = AllianceFlipUtil.apply(siteConstants.topLeftCenterPoint.toTranslation2d());
                } else {
                        target = AllianceFlipUtil.apply(siteConstants.topRightCenterPoint.toTranslation2d());
                }
                double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

                // 2. 砲塔場地速度計算 (✨ 修正了外積的正負號問題)
                ChassisSpeeds robotVelocity = drive.getFieldVelocity();
                if (AllianceFlipUtil.shouldFlip()) {
                        robotVelocity = new ChassisSpeeds(
                                        -robotVelocity.vxMetersPerSecond,
                                        -robotVelocity.vyMetersPerSecond,
                                        robotVelocity.omegaRadiansPerSecond);
                }

                Translation2d turretOffsetField = ShooterConstants.robotToLLauch.getTranslation().toTranslation2d()
                                .rotateBy(estimatedPose.getRotation());

                double turretVelocityX = robotVelocity.vxMetersPerSecond
                                - (robotVelocity.omegaRadiansPerSecond * turretOffsetField.getY());
                double turretVelocityY = robotVelocity.vyMetersPerSecond
                                + (robotVelocity.omegaRadiansPerSecond * turretOffsetField.getX());

                // 3. 迭代預測虛擬目標點 (Lookahead Pose)
                double timeOfFlight = 0.0;
                Pose2d lookaheadPose = turretPosition;
                double lookaheadTurretToTargetDistance = turretToTargetDistance;

                for (int i = 0; i < 5; i++) {
                        timeOfFlight = ToAillancetimeOfFlightMap.get(lookaheadTurretToTargetDistance);

                        // ✨ 升級：空氣阻力補償 (Effective TOF)
                        double effectiveTOF = (1 - Math.exp(-timeOfFlight * linearDragTimeConstant))
                                        / linearDragTimeConstant;

                        double offsetX = turretVelocityX * effectiveTOF;
                        double offsetY = turretVelocityY * effectiveTOF;

                        lookaheadPose = new Pose2d(
                                        turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                                        turretPosition.getRotation());

                        lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
                }

                // 4. 計算最終對正角度
                Translation2d vectorToTarget = target.minus(lookaheadPose.getTranslation());
                Rotation2d targetFieldAngle = vectorToTarget.getAngle();

                if (AllianceFlipUtil.shouldFlip()) {
                        targetFieldAngle = Rotation2d.fromDegrees(targetFieldAngle.getDegrees() - 180.0);
                }

                return new ShootingState(
                                targetFieldAngle,
                                hoodMap.get(lookaheadTurretToTargetDistance),
                                rollMap.get(lookaheadTurretToTargetDistance));
        }

}