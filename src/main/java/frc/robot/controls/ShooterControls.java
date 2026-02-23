package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.Mode;

public class ShooterControls {
    private final ShooterSubsystem shooterSubsystem;
    private final CommandXboxController controller;

    private double flywheelSpeedOffset = 0.0;
    private double backboardPositionOffset = 0.0;

    public ShooterControls(ShooterSubsystem shooterSubsystem, CommandXboxController controller) {
        this.shooterSubsystem = shooterSubsystem;
        this.controller = controller;
    }

    public Command defaultShooterCommand() {
        return Commands.run(() -> {
                // System.out.println("running");
                if (controller.getRightTriggerAxis() < 0.1) {
                    shooterSubsystem.setConveyorSpeedByRPS(0);
                    shooterSubsystem.setFlywheelSpeedByRPS(0);
                    return;
                }
                double flywheelSpeed;
                double backboardPosition;
                double conveyorSpeed;
                flywheelSpeed = calculateFlywheelSpeedOnlywithOffset(20);
                backboardPosition = calculateBackboardPositionOnlywithOffset(100);
                conveyorSpeed = controller.getRightTriggerAxis() * Constants.Shooter.conveyorSpeedkP;

                // double flywheelSpeed = calculateFlywheelSpeed(0);
                // double BackboardPosition = calculateBackboardPosition(0);
                // shooterSubsystem.setFlywheelSpeedByRPS(flywheelSpeed);
                // shooterSubsystem.setConveyorSpeedByRPS(conveyorSpeed);
                shooterSubsystem.setBackboardPosition(backboardPosition);
        }, shooterSubsystem);
    }

    // public Command resetBackboardCommand(){
    //     return 
    //         Commands.runOnce(()->{
    //             shooterSubsystem.backboardMotor.set(-0.1);
    //             shooterSubsystem.currentBackboardInalyzeMode = Mode.Inalyzing;}, shooterSubsystem)
    //         .andThen(
    //             Commands.waitSeconds(0.5))
    //         .andThen(Commands.runOnce(()->{
    //             shooterSubsystem.backboardMotor.set(0);
    //             shooterSubsystem.backboardEncoder.reset();
    //             shooterSubsystem.backboardPID.reset(0);}, shooterSubsystem))
    //         .andThen(
    //             Commands.waitSeconds(1))
    //         .andThen(Commands.runOnce(()->{
    //             shooterSubsystem.currentBackboardInalyzeMode = Mode.Inalyzed;}, shooterSubsystem));
    // }

    public Command resetBackboardCommand(){
        return 
            Commands.runOnce(()->{
                shooterSubsystem.backboardEncoder.reset();
                shooterSubsystem.backboardPID.reset(0);}, shooterSubsystem);
    }

    //打表参数
    private static final double[][] FLYWHEEL_TABLE = {
    {1.5, 0.42},
    {2.0, 0.46},
    {2.5, 0.51},
    {3.0, 0.57},
    {3.5, 0.64},
    {4.0, 0.72}
    };

    //线性插值函数
    private static double lerp(double x0, double y0, double x1, double y1, double x)  {
    if (Math.abs(x1 - x0) < 1e-9) {
        return y0;
    }
    double t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
    }
    private double calculateFlywheelSpeedOnlywithOffset(double defaultSpeed) {
        return defaultSpeed + flywheelSpeedOffset;
    }
    private double calculateBackboardPositionOnlywithOffset(double defaultPosition) {
        return defaultPosition + backboardPositionOffset;
    }

    private double calculateFlywheelSpeed(double distance) {
        // 小于最小距离：用第一个点
        if (distance <= FLYWHEEL_TABLE[0][0]) {
        return FLYWHEEL_TABLE[0][1] + flywheelSpeedOffset;
        }
        // 大于最大距离：用最后一个点
        int last = FLYWHEEL_TABLE.length - 1;
        if (distance >= FLYWHEEL_TABLE[last][0]) {
        return FLYWHEEL_TABLE[last][1] + flywheelSpeedOffset;
        }
        // 查找区间并插值
        for (int i = 0; i < FLYWHEEL_TABLE.length - 1; i++) {
            double d0 = FLYWHEEL_TABLE[i][0];
            double s0 = FLYWHEEL_TABLE[i][1];
            double d1 = FLYWHEEL_TABLE[i + 1][0];
            double s1 = FLYWHEEL_TABLE[i + 1][1];
            if (distance >= d0 && distance <= d1) {
                return lerp(d0, s0, d1, s1, distance) + flywheelSpeedOffset;
            }
        }
        return 0;
    }
    
    private double calculateBackboardPosition(double distance) {
        double BackboardPosition;
        // 距离阈值（单位自己按distance进行修改）
        final double DISTANCE_NEAR = 2.0;   // 近距离阈值
        final double DISTANCE_MID  = 4.0;   // 中距离阈值
        if (distance < DISTANCE_NEAR) {
        BackboardPosition = Constants.ShooterCalculation.positionNear;
        } 
        else if (distance < DISTANCE_MID) {
        BackboardPosition = Constants.ShooterCalculation.positionMid;
        } 
        else {
        BackboardPosition = Constants.ShooterCalculation.postionFar;
        }
        BackboardPosition += backboardPositionOffset;
        return BackboardPosition;
    }

    public void adjustFlywheelSpeedOffset(double offset) {
        flywheelSpeedOffset += offset;
    }

    public void adjustBackboardRateOffset(double offset) {
        backboardPositionOffset += offset;
    }
}