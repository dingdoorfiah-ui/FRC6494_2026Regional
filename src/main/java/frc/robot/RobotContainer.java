package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.controls.DriveControls;
import frc.robot.controls.ShooterControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.tuning.ConfigTalonFXMotorTuner;
import frc.robot.tuning.ConfigTalonFXSMotorTuner;
import frc.robot.tuning.DriveGainsTuner;
import frc.robot.utils.LedBindings;

public class RobotContainer {

  /* ====================== */
  /*         手柄            */
  /* ====================== */

  // 只使用一个 Xbox 手柄（端口 0）
  private final CommandXboxController controller = new CommandXboxController(0);

  /* ====================== */
  /*         子系统           */
  /* ====================== */

  // Phoenix TunerX 生成的底盘
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // LED 子系统
  private final LEDSubsystem leds = new LEDSubsystem(0, 72 ,0); // 总长度=72+72         

  /* ====================== */
  /*     控制封装/日志        */
  /* ====================== */

  private final DriveControls driveControls = new DriveControls(drivetrain, controller);
  private final ShooterControls shooterControls = new ShooterControls(shooterSubsystem, controller);

  private final DriveGainsTuner driveGainsTuner = new DriveGainsTuner(drivetrain);
  private ConfigTalonFXMotorTuner configFlywheelTuner = new ConfigTalonFXMotorTuner(shooterSubsystem.flywheelMotorLeft, "flywheel", Constants.Shooter.flyWheelSlot0Configs);
  private ConfigTalonFXMotorTuner configConveyorTuner = new ConfigTalonFXMotorTuner(shooterSubsystem.conveyorMotor, "conveyor", Constants.Shooter.conveyorSlot0Configs);
  private ConfigTalonFXSMotorTuner configBackboardTuner = new ConfigTalonFXSMotorTuner(shooterSubsystem.backboardMotor, "backboard", Constants.Shooter.backboardSlot0Configs);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    SignalLogger.enableAutoLogging(false);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configueSwerve();
    configueShooter();
    bingdingLED();
  }

  /* ====================== */
  /*        底盘配置          */
  /* ====================== */

  private void configueSwerve() {
    Pose2d startingPose = Constants.StartingPoints.Red.Point2;
    drivetrain.resetPose(startingPose);

    // 默认驾驶命令
    drivetrain.setDefaultCommand(driveControls.defaultDriveCommand());

    // Disabled 时进入 idle（防止模块乱动）
    RobotModeTriggers.disabled().whileTrue(driveControls.idleCommand());

    
    //左保险：按住刹车
    controller.leftBumper()
      .onTrue(Commands.runOnce(driveControls::emergencyStop))
      .whileTrue(driveControls.brakeWhileHeld());

    // Back：重置场地坐标系
    controller.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    
    // Start：切换场地/车体坐标系
    controller.start().onTrue(Commands.runOnce(driveControls::toggleDriveFrame));

    // 右保险：按住加速（Boost）
    controller.rightBumper()
        .onTrue(Commands.runOnce(() -> driveControls.setBoostEnabled(true)))
        .onFalse(Commands.runOnce(() -> driveControls.setBoostEnabled(false)));

    controller.a().whileTrue(drivetrain.runOnce(drivetrain::forceUsingLimelightMT2));

  }

  private void configueShooter(){
    shooterSubsystem.setDefaultCommand(shooterControls.defaultShooterCommand());

    controller.povLeft().onTrue(Commands.runOnce(() -> shooterControls.adjustFlywheelSpeedOffset(-1)));
    controller.povRight().onTrue(Commands.runOnce(() -> shooterControls.adjustFlywheelSpeedOffset(1)));
    controller.povUp().onTrue(Commands.runOnce(() -> shooterControls.adjustBackboardRateOffset(100)));
    controller.povDown().onTrue(Commands.runOnce(() -> shooterControls.adjustBackboardRateOffset(-100)));
  }

  /* ====================== */
  /*        LED 绑定          */
  /* ====================== */

  private void bingdingLED() {
    LedBindings.bindModeIndicators(leds);
  }
  
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }

  public void tunerPeriodic() {
    driveGainsTuner.periodic();
    configFlywheelTuner.periodic(() -> shooterSubsystem.applyLeftConfigurationToRight());
    configConveyorTuner.periodic(null);
    configBackboardTuner.periodic();
    shooterSubsystem.setRightFollowLeft();
  }

}
