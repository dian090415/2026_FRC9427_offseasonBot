package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class ShooterConstants {
    public static Transform3d robotToRightFlywheel = new Transform3d(0.151136, 0.29845 + 0.03195, 0.487484,
            Rotation3d.kZero);
    public static Transform3d robotToLeftFlywheel = new Transform3d(0.151136, -0.29845 + 0.03195, 0.487484,
            Rotation3d.kZero);
    public static Transform3d robotToLLauch = new Transform3d(0.151136, 0.0, 0.487484,
            Rotation3d.kZero);

    public static final Angle Hood_MAX_LIMIT = Degree.of(45); // 上限75
    public static final Angle Hood_MIN_LIMIT = Degree.of(1); // 下限25

    public static final double Hood_GEAR_RATIO = 14.75;//24 / 16 * 118 / 12

    public static final double Flywheel_GEAR_RATIO = 1.0 / 1.4;

    public static final double Tigger_GEAR_RATIO = 1.0 / 3.0;

    public static final class TriggerConstants {
        public static final Current TRIGGER_STATOR_CURRENT_LIMIT = Amp.of(40);
        public static final Current TRIGGER_SUPPLY_CURRENT_LIMIT = Amp.of(45);
    }
}