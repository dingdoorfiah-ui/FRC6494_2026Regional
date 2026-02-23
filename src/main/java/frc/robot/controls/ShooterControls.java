package frc.robot.controls;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterControls {
    private final ShooterSubsystem shooterSubsystem;
    private final CommandXboxController controller;

    private double flywheelSpeedOffset = 0.0;
    private double backboardPositionOffset = 0.0;
    private NetworkTable shooterControlTable;
    public ShooterControls(ShooterSubsystem shooterSubsystem, CommandXboxController controller) {
        this.shooterSubsystem = shooterSubsystem;
        this.controller = controller;

        shooterControlTable = NetworkTableInstance.getDefault().getTable("ShooterControl");
    }

    public Command defaultShooterCommand() {
        return Commands.run(() -> {
                double flywheelSpeed;
                double backboardPosition;
                double conveyorSpeed;
                // flywheelSpeed = calculateFlywheelSpeedOnlywithOffset(60);
                var x = shooterSubsystem.getDistance();
                flywheelSpeed = 0.6956596811733782*x*x*x*x*x*x*x-14.164408860183267*x*x*x*x*x*x+119.7115375240918*x*x*x*x*x-543.0984876677477*x*x*x*x+1425.1928543240738*x*x*x-2156.199296490315*x*x+1736.568139600292*x-516.097598057351;
                flywheelSpeed += flywheelSpeedOffset;
                backboardPosition = -122.11889023593778*x*x*x*x*x*x*x+2453.385583841308*x*x*x*x*x*x-20553.05413872581*x*x*x*x*x+92963.33175750206*x*x*x*x-244905.25916147238*x*x*x+375150.1363997868*x*x-308031.51715612336*x+104203.52690842684;
                backboardPosition += backboardPositionOffset;
                conveyorSpeed = Constants.Shooter.conveyorSpeed;
                shooterControlTable.getEntry("flywheelTargetSpeed").setDouble(flywheelSpeed);
                shooterControlTable.getEntry("conveyerTargetSpeed").setDouble(conveyorSpeed);
                shooterControlTable.getEntry("backboardTargetPosition").setDouble(backboardPosition);
                // System.out.println("running");
                if (controller.getRightTriggerAxis() < 0.1) {
                    flywheelSpeed = 0;
                    conveyorSpeed = 0;
                }

                // double flywheelSpeed = calculateFlywheelSpeed(0);
                // double BackboardPosition = calculateBackboardPosition(0);
                shooterSubsystem.setFlywheelSpeedByRPS(flywheelSpeed);
                shooterSubsystem.setConveyorSpeedByRPS(conveyorSpeed);
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

    public void outputInfo(){
        double currentConveyorSpeed = shooterSubsystem.conveyorMotor.getVelocity().getValueAsDouble();
        double currentFlywheelSpeed = shooterSubsystem.flywheelMotorLeft.getVelocity().getValueAsDouble();
        double currentBackboardRate = shooterSubsystem.getBackboardPosition();
        System.out.println("Conveyor: " + currentConveyorSpeed + "\t Flywheel: " + currentFlywheelSpeed + "\t Backboard:" + currentBackboardRate);
    }
    public Command resetBackboardCommand(){
        return 
            Commands.runOnce(()->{
                shooterSubsystem.backboardEncoder.reset();
                shooterSubsystem.backboardPID.reset(0);}, shooterSubsystem);
    }

    //打表参数
    // private static final double[][] FLYWHEEL_TABLE = {
    // {1.5, 0.42},
    // {2.0, 0.46},
    // {2.5, 0.51},
    // {3.0, 0.57},
    // {3.5, 0.64},
    // {4.0, 0.72}
    // };

    // //线性插值函数
    // private static double lerp(double x0, double y0, double x1, double y1, double x)  {
    // if (Math.abs(x1 - x0) < 1e-9) {
    //     return y0;
    // }
    // double t = (x - x0) / (x1 - x0);
    // return y0 + t * (y1 - y0);
    // }
    // private double calculateFlywheelSpeedOnlywithOffset(double defaultSpeed) {
    //     return defaultSpeed + flywheelSpeedOffset;
    // }
    // private double calculateBackboardPositionOnlywithOffset(double defaultPosition) {
    //     return defaultPosition + backboardPositionOffset;
    // }


    // private double calculateFlywheelSpeed(double distance) {
    //     // 小于最小距离：用第一个点
    //     if (distance <= FLYWHEEL_TABLE[0][0]) {
    //     return FLYWHEEL_TABLE[0][1] + flywheelSpeedOffset;
    //     }
    //     // 大于最大距离：用最后一个点
    //     int last = FLYWHEEL_TABLE.length - 1;
    //     if (distance >= FLYWHEEL_TABLE[last][0]) {
    //     return FLYWHEEL_TABLE[last][1] + flywheelSpeedOffset;
    //     }
    //     // 查找区间并插值
    //     for (int i = 0; i < FLYWHEEL_TABLE.length - 1; i++) {
    //         double d0 = FLYWHEEL_TABLE[i][0];
    //         double s0 = FLYWHEEL_TABLE[i][1];
    //         double d1 = FLYWHEEL_TABLE[i + 1][0];
    //         double s1 = FLYWHEEL_TABLE[i + 1][1];
    //         if (distance >= d0 && distance <= d1) {
    //             return lerp(d0, s0, d1, s1, distance) + flywheelSpeedOffset;
    //         }
    //     }
    //     return 0;
    // }
    
    // private double calculateBackboardPosition(double distance) {
    //     double BackboardPosition;
    //     // 距离阈值（单位自己按distance进行修改）
    //     final double DISTANCE_NEAR = 2.0;   // 近距离阈值
    //     final double DISTANCE_MID  = 4.0;   // 中距离阈值
    //     if (distance < DISTANCE_NEAR) {
    //     BackboardPosition = Constants.ShooterCalculation.positionNear;
    //     } 
    //     else if (distance < DISTANCE_MID) {
    //     BackboardPosition = Constants.ShooterCalculation.positionMid;
    //     } 
    //     else {
    //     BackboardPosition = Constants.ShooterCalculation.postionFar;
    //     }
    //     BackboardPosition += backboardPositionOffset;
    //     return BackboardPosition;
    // }

    public void adjustFlywheelSpeedOffset(double offset) {
        flywheelSpeedOffset += offset;
    }

    public void adjustBackboardRateOffset(double offset) {
        backboardPositionOffset += offset;
    }
}