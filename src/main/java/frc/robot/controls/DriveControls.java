package frc.robot.controls;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveMode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveControls {
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driver;

    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(edu.wpi.first.units.Units.RadiansPerSecond);

    // 默认速度比例（平时开车）
    private double normalDriveScale = 0.3;
    private double normalTurnScale  = 0.7;

    // 加速模式比例（按住 RB）
    private double boostDriveScale = 1.0;
    private double boostTurnScale  = 1.0;

    private boolean boostEnabled = false;

    private final SlewRateLimiter vxLimiter = new SlewRateLimiter(5.0);   // m/s^2 等效（调）
    private final SlewRateLimiter vyLimiter = new SlewRateLimiter(5.0);
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(9.0); // rad/s^2 等效（调）

    // ====== 类成员变量里新增：记录上一次输出 ======
    private double lastVx = 0.0;
    private double lastVy = 0.0;
    private double lastOmega = 0.0;

    private NetworkTable autoControlNetworkTable;
    
    public void setBoostEnabled(boolean enabled) {
        boostEnabled = enabled;
    }

    // 新增：当前是否使用场地坐标系（默认 true）
    private boolean fieldCentricEnabled = true;

    private final SwerveRequest.FieldCentric fieldCentric =
        new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.05)
            .withRotationalDeadband(maxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.RobotCentric robotCentric =
    new SwerveRequest.RobotCentric()
        .withDeadband(maxSpeed * 0.05)
        .withRotationalDeadband(maxAngularRate * 0.05)
        .withDriveRequestType(DriveRequestType.Velocity);
    
    public DriveControls(CommandSwerveDrivetrain drivetrain, CommandXboxController driver) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        autoControlNetworkTable = NetworkTableInstance.getDefault().getTable("AutoControl");
    }
    
    public void toggleDriveFrame() {
        fieldCentricEnabled = !fieldCentricEnabled;
    }
    
    public boolean isFieldCentricEnabled() {
        return fieldCentricEnabled;
    }
    public DriveMode getCurrentDriveMode() {
        return fieldCentricEnabled ? DriveMode.FIELD_CENTRIC : DriveMode.ROBOT_CENTRIC;
    }
    /** 底盘的默认驾驶命令（teleop 期间持续执行） */
    public Command defaultDriveCommand() {
        return drivetrain.applyRequest(() -> {
        double driveScale = boostEnabled ? boostDriveScale : normalDriveScale;
        double turnScale  = boostEnabled ? boostTurnScale  : normalTurnScale;

        double vx = -driver.getLeftY() * maxSpeed * driveScale;
        double vy = -driver.getLeftX() * maxSpeed * driveScale;
        double omega = -driver.getRightX() * maxAngularRate * turnScale;

        // ====== execute 内替换 limiter：只限加速、不限减速 ======
        double limitedVx = limitAccelOnly(vx, lastVx, vxLimiter);
        double limitedVy = limitAccelOnly(vy, lastVy, vyLimiter);
        double limitedOmega = limitAccelOnly(omega, lastOmega, omegaLimiter);

        lastVx = limitedVx;
        lastVy = limitedVy;
        lastOmega = limitedOmega;

        vx = limitedVx;
        vy = limitedVy;
        omega = limitedOmega;
        
        double[] distanceAndRotation = calculateDistanceAndRotationToHub();
        // 将距离值写入 NetworkTable
        autoControlNetworkTable.getEntry("distanceToHub").setDouble(distanceAndRotation[0]);
        autoControlNetworkTable.getEntry("angleDifferenceToHub").setDouble(distanceAndRotation[1]);

        if(Constants.AutoRotation.autoRotationToHubEnabled) {
            double autoRotationPower = driver.getLeftTriggerAxis();
            if(autoRotationPower > 0.1) {
                autoControlNetworkTable.getEntry("autoRotationAvailable").setBoolean(true);

                double angleDifference = distanceAndRotation[1];

                // Calculate rotation speed using the angle difference
                omega = calculateRotationSpeedFromRotationAngle(angleDifference, autoRotationPower);

                autoControlNetworkTable.getEntry("autoRotationRate").setDouble(omega);
            } else {
                autoControlNetworkTable.getEntry("autoRotationAvailable").setBoolean(false);
            }
        }

      

        if (fieldCentricEnabled) {
            return fieldCentric
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega);
        } else {
            return robotCentric
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega);
        }
        }, () -> fieldCentricEnabled ? DriveMode.FIELD_CENTRIC : DriveMode.ROBOT_CENTRIC);
    }
    
    // Calculate distance and rotation (angle difference)
    public double[] calculateDistanceAndRotationToHub() {
        Pose2d currentPose = drivetrain.getState().Pose; // 获取机器人当前的位置和角度
        double targetX = Constants.Field.RedHubPositionX;  // 目标 X 坐标
        double targetY = Constants.Field.RedHubPositionY;  // 目标 Y 坐标

        // 计算目标角度（相对于场地坐标系）
        double deltaX = targetX - currentPose.getX();
        double deltaY = targetY - currentPose.getY();
        double targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX)); // 转换为度

        // 当前机器人角度
        double currentAngle = currentPose.getRotation().getDegrees();
        double angleDifference = targetAngle - currentAngle;
        // Normalize angle difference to the range [-180, 180]
        if (angleDifference > 180) {
            angleDifference -= 360;
        } else if (angleDifference < -180) {
            angleDifference += 360;
        }

        // 计算距离
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                // 计算距离和旋转角度
        double[] distanceAndRotation = new double[]{distance, angleDifference};

        // 返回距离和角度差
        return distanceAndRotation;
    }

    // Calculate rotation speed from rotation angle difference
    public double calculateRotationSpeedFromRotationAngle(double angleDifference, double autoRotationPower) {
        double calDifference = angleDifference;

        // Optional: Apply teleop offset if enabled
        if (Constants.AutoRotation.teleopOffsetEnabled) {
            double teleopOffset = -driver.getRightX();
            calDifference += teleopOffset * Constants.AutoRotation.teleopOffsetkP;
        }

        // If angle difference is within threshold, don't rotate
        if (Math.abs(calDifference) > 1.0) {
            return calDifference * autoRotationPower * Constants.AutoRotation.autoRotationToHubkP;
        } else {
            return 0;
        }
    }



        /**
     * 只限制“加速”（幅值变大），不限制“减速”（幅值变小）
     * 这样松杆/刹车会更及时，同时 boost 仍然能抑制电流冲击
     */
    private double limitAccelOnly(double target, double last, SlewRateLimiter limiter) {
        // 如果正在减速（幅值变小），直接放行到目标值
        if (Math.abs(target) < Math.abs(last)) {
        limiter.reset(target);  // 同步 limiter 内部状态，防止下一次突变
        return target;
        }
        // 否则是加速，使用 limiter
        return limiter.calculate(target);
    }

        /** 紧急停止：清零限速器，让速度输出立刻归零 */
    public void emergencyStop() {
        vxLimiter.reset(0.0);
        vyLimiter.reset(0.0);
        omegaLimiter.reset(0.0);

        lastVx = 0.0;
        lastVy = 0.0;
        lastOmega = 0.0;
    }

    public Command idleCommand() {
        final var idle = new SwerveRequest.Idle();
        return drivetrain.applyRequest(() -> idle).ignoringDisable(true);
    }
    
    public Command brakeWhileHeld() {
        final var brake = new SwerveRequest.SwerveDriveBrake();
        return drivetrain.applyRequest(() -> brake);
    }
    

    public SwerveRequest.SwerveDriveBrake brakeRequest() {
        return new SwerveRequest.SwerveDriveBrake();
    }

    public SwerveRequest.Idle idleRequest() {
        return new SwerveRequest.Idle();
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getMaxAngularRate() {
        return maxAngularRate;
    }

}
