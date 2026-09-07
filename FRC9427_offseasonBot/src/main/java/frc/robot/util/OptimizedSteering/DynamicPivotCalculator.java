package frc.robot.util.OptimizedSteering;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants; // 請替換成你實際放 moduleLocations 的地方

public class DynamicPivotCalculator {

    // 啟動 ICR 模式的最低旋轉速度 (rad/s)
    private static final double OMEGA_THRESHOLD = 0.2;
    // 啟動方向模式的最低平移速度 (m/s)
    private static final double VELOCITY_THRESHOLD = 0.1;

    /**
     * 根據當前期望速度，動態計算最合理的旋轉圓心 (角塊位置)
     *
     * @param speeds 駕駛員預期的車體速度
     * @return 最適合當作旋轉中心的坐標 (Translation2d)
     */
    public static Translation2d getBestPivotWheel(ChassisSpeeds speeds) {
        Translation2d[] modules = DriveConstants.moduleLocations;
        Translation2d bestPivot = new Translation2d(); // 預設為中心 (0, 0)

        // 【方法 3：最低摩擦優先 (ICR)】- 當旋轉速度夠快時觸發
        if (Math.abs(speeds.omegaRadiansPerSecond) > OMEGA_THRESHOLD) {
            
            // 計算瞬時旋轉中心 (ICR) 的坐標
            // 根據 V = ω × R 反推： X_icr = Vy / ω, Y_icr = -Vx / ω
            double icrX = speeds.vyMetersPerSecond / speeds.omegaRadiansPerSecond;
            double icrY = -speeds.vxMetersPerSecond / speeds.omegaRadiansPerSecond;
            Translation2d icr = new Translation2d(icrX, icrY);

            double minDistance = Double.POSITIVE_INFINITY;

            // 尋找距離 ICR 最近的輪子
            for (Translation2d module : modules) {
                double distance = module.getDistance(icr);
                if (distance < minDistance) {
                    minDistance = distance;
                    bestPivot = module;
                }
            }
            return bestPivot;
        } 
        
        // 【方法 2：方向優先 (甩尾/閃避)】- 當只有平移沒有旋轉時觸發
        else if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > VELOCITY_THRESHOLD) {
            
            Translation2d velocityVector = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
            double maxDotProduct = Double.NEGATIVE_INFINITY;

            // 利用內積 (Dot Product) 找出與前進方向最平行的輪子
            for (Translation2d module : modules) {
                double dotProduct = (module.getX() * velocityVector.getX()) + (module.getY() * velocityVector.getY());
                if (dotProduct > maxDotProduct) {
                    maxDotProduct = dotProduct;
                    bestPivot = module;
                }
            }
            return bestPivot;
        }

        // 如果速度很慢，回歸車體正中央
        return bestPivot;
    }

    /**
     * 前饋補償計算：消除因為偏移圓心所造成的車體甩尾漂移
     * 
     * @param centerSpeeds 原始預期速度
     * @param pivot 決定的圓心坐標
     * @return 補償後的最終速度
     */
    public static ChassisSpeeds calculateOffsetCompensation(ChassisSpeeds centerSpeeds, Translation2d pivot) {
        if (Math.abs(centerSpeeds.omegaRadiansPerSecond) < 0.01 || (pivot.getX() == 0.0 && pivot.getY() == 0.0)) {
            return centerSpeeds;
        }

        // Vx_offset = -ω * Y_r
        // Vy_offset =  ω * X_r
        double omega = centerSpeeds.omegaRadiansPerSecond;
        double vxCompensation = -omega * pivot.getY();
        double vyCompensation =  omega * pivot.getX();

        return new ChassisSpeeds(
                centerSpeeds.vxMetersPerSecond + vxCompensation,
                centerSpeeds.vyMetersPerSecond + vyCompensation,
                omega
        );
    }
}