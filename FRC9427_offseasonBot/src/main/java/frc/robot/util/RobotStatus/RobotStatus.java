package frc.robot.util.RobotStatus;

import java.util.ArrayList;
import java.util.EventObject;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.siteConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.util.FIeldHelper.AllianceFlipUtil;
import frc.robot.util.RobotEvent.Event.*;

public class RobotStatus extends SubsystemBase {

    public EventObject eventObject = new EventObject(getClass());

    private static final double BLUE_ZONE_LIMIT = 5.50;
    private static final double RED_ZONE_START = FieldConstants.fieldLength - 5.50;
    private static final double MID_Y = FieldConstants.fieldWidth / 2.0;

    public final CommandSwerveDrivetrain drive;

    private final List<NeedResetPoseEvent> needResetPoseEvents = new ArrayList<>();
    private final List<InTrench> inTrenchEvents = new ArrayList<>();
    private final List<NotInTrench> notInTrenchEvents = new ArrayList<>();

    public enum Area {
        CENTER,
        BlueAlliance,
        RedAlliance
    }

    public enum VerticalSide {
        TOP,
        BOTTOM
    }

    public boolean NeedResetPose = false;

    private boolean m_wasClimbing = false;

    public boolean SafeHood = false;

    // 新增：用於狀態改變檢測 (Edge Detection)
    private boolean m_lastInTrench = false;
    // 新增：用於降低 Log 頻率
    private int logCounter = 0;

    public RobotStatus(CommandSwerveDrivetrain drive) {
        this.drive = drive;
    }

    public VerticalSide getVerticalSide() {
        double Y = drive.getPose2d().getY();
        return (Y > MID_Y) ? VerticalSide.TOP : VerticalSide.BOTTOM;
    }

    public Area getArea() {
        double x = drive.getPose2d().getX();
        if (x < BLUE_ZONE_LIMIT)
            return Area.BlueAlliance;
        if (x > RED_ZONE_START)
            return Area.RedAlliance;
        return Area.CENTER;
    }

    // --- 事件註冊區 ---
    public void TriggerNeedResetPoseEvent(NeedResetPoseEvent event) {
        needResetPoseEvents.add(event);
    }

    public void TriggerInTrench(InTrench event) {
        inTrenchEvents.add(event);
    }

    public void TriggerNotInTrench(NotInTrench event) {
        notInTrenchEvents.add(event);
    }

    /**
     * 核心邏輯：處理爬升後的 Pose 重置
     */
    public void updateOdometerStatus() {
        boolean isNowClimbing = this.drive.isClimbing();

        if (m_wasClimbing && !isNowClimbing) {
            for (NeedResetPoseEvent listener : needResetPoseEvents) {
                listener.NeedResetPose();
            }
        }
        m_wasClimbing = isNowClimbing;
    }

    /**
     * 判斷是否在我方聯盟區域
     * (已包含 Null Safety)
     */
    public boolean isInMyAllianceZone() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isEmpty())
            return false;

        Area currentArea = getArea();
        if (ally.get() == Alliance.Blue) {
            return currentArea == Area.BlueAlliance;
        } else {
            return currentArea == Area.RedAlliance;
        }
    }

    /**
     * *** 效能修復核心 ***
     * 只在狀態改變時觸發事件，而不是每個 Loop 都觸發。
     */
    public void updateTrenchStatus() {
        boolean isNowInTrench = isInTrench();

        // 只有當狀態跟上一次不一樣時 (State Changed)，才執行動作
        if (isNowInTrench != m_lastInTrench) {
            if (isNowInTrench) {
                // 剛進入 Trench
                for (InTrench listener : inTrenchEvents) {
                    listener.InTrench();
                }
            } else {
                // 剛離開 Trench
                for (NotInTrench listener : notInTrenchEvents) {
                    listener.NotInTrench();
                }
            }
            // 狀態改變時，強制記錄 Log
            Logger.recordOutput("RobotStatus/InTrench", isNowInTrench);
        }

        m_lastInTrench = isNowInTrench;
    }

public boolean isInTrench() {
        var currentPose = drive.getPose2d();
        
        Pose2d currentPose_Left = currentPose.transformBy(
                new Transform2d(
                        ShooterConstants.robotToLeftFlywheel.getTranslation().toTranslation2d(),
                        ShooterConstants.robotToLeftFlywheel.getRotation().toRotation2d()));
                        
        Pose2d currentPose_Right = currentPose.transformBy(
                new Transform2d(
                        ShooterConstants.robotToRightFlywheel.getTranslation().toTranslation2d(),
                        ShooterConstants.robotToRightFlywheel.getRotation().toRotation2d()));

        // 檢查一般區域 (藍方)
        if (isRobotWidthInArea(currentPose_Left, currentPose_Right, 
                siteConstants.Right_TRENCHE_Pose1, siteConstants.Right_TRENCHE_Pose2, siteConstants.Right_TRENCHE_Pose3))
            return true;
            
        if (isRobotWidthInArea(currentPose_Left, currentPose_Right, 
                siteConstants.Left_TRENCHE_Pose1, siteConstants.Left_TRENCHE_Pose2, siteConstants.Left_TRENCHE_Pose3))
            return true;

        // 檢查翻轉區域 (紅方)
        if (isRobotWidthInArea(currentPose_Left, currentPose_Right,
                AllianceFlipUtil.Needapply(siteConstants.Right_TRENCHE_Pose1),
                AllianceFlipUtil.Needapply(siteConstants.Right_TRENCHE_Pose2),
                AllianceFlipUtil.Needapply(siteConstants.Right_TRENCHE_Pose3)))
            return true;

        if (isRobotWidthInArea(currentPose_Left, currentPose_Right,
                AllianceFlipUtil.Needapply(siteConstants.Left_TRENCHE_Pose1),
                AllianceFlipUtil.Needapply(siteConstants.Left_TRENCHE_Pose2),
                AllianceFlipUtil.Needapply(siteConstants.Left_TRENCHE_Pose3)))
            return true;

        return false;
    }

    /**
     * 檢查從 Left 到 Right 的連線線段，是否有任何一點進入了指定的矩形區域中
     */
    private boolean isRobotWidthInArea(Pose2d leftPose, Pose2d rightPose, Pose2d... corners) {
        if (corners == null || corners.length == 0)
            return false;

        // 1. 算出目標區域的判定框 (Bounding Box)
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;

        for (Pose2d corner : corners) {
            if (corner != null) {
                if (corner.getX() < minX) minX = corner.getX();
                if (corner.getX() > maxX) maxX = corner.getX();
                if (corner.getY() < minY) minY = corner.getY();
                if (corner.getY() > maxY) maxY = corner.getY();
            }
        }

        double x1 = leftPose.getX();
        double y1 = leftPose.getY();
        double x2 = rightPose.getX();
        double y2 = rightPose.getY();

        // 2. 狀況 A：如果左邊界或右邊界「已經在框內」，直接判定為 true
        if ((x1 >= minX && x1 <= maxX && y1 >= minY && y1 <= maxY) ||
            (x2 >= minX && x2 <= maxX && y2 >= minY && y2 <= maxY)) {
            return true;
        }

        // 3. 狀況 B：快速排除 (Quick Reject) - 如果整台機器人的寬度完全在判定框的上下左右範圍之外，絕對不可能碰到
        if (Math.min(x1, x2) > maxX || Math.max(x1, x2) < minX ||
            Math.min(y1, y2) > maxY || Math.max(y1, y2) < minY) {
            return false;
        }

        // 4. 狀況 C：線段切割 (Cross Product Check) - 機器人跨越了邊界，線段與矩形相交
        double dx = x2 - x1;
        double dy = y2 - y1;

        // 將矩形的 4 個頂點帶入直線方程式
        double c1 = (minX - x1) * dy - (minY - y1) * dx;
        double c2 = (maxX - x1) * dy - (minY - y1) * dx;
        double c3 = (maxX - x1) * dy - (maxY - y1) * dx;
        double c4 = (minX - x1) * dy - (maxY - y1) * dx;

        // 如果矩形的 4 個角都在這條線的「同一側」(同正或同負)，代表線沒有穿過矩形
        if ((c1 > 0 && c2 > 0 && c3 > 0 && c4 > 0) || (c1 < 0 && c2 < 0 && c3 < 0 && c4 < 0)) {
            return false;
        }

        // 通過以上所有檢查，代表這條線確實切進了矩形範圍內！
        return true;
    }

    @Override
    public void periodic() {
        // 1. 更新狀態與事件觸發
        // this.updateOdometerStatus();
        // this.updateTrenchStatus(); // 改名後的 trench 事件處理

        // // 2. 優化 Logging：每 10 個 Loop (0.2秒) 更新一次，或是在狀態改變時更新
        // // 這樣可以把 22ms 降到 < 1ms
        // logCounter++;
        // if (logCounter >= 25) { // 每 0.5 秒更新一次狀態 Log
        //     Logger.recordOutput("RobotStatus/InMyAllianceZone", isInMyAllianceZone());
        //     Logger.recordOutput("RobotStatus/Area", getArea().toString());
        //     Logger.recordOutput("RobotStatus/VerticalSide", getVerticalSide().toString());
        //     logCounter = 0;
        // }
    }
}