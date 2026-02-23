package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.LEDSubsystem;

public final class LedBindings {
  private LedBindings() {}

  public static void bindModeIndicators(LEDSubsystem leds) {
    // Disabled：左右红色呼吸（允许在禁用状态运行）
    RobotModeTriggers.disabled().onTrue(
        Commands.runOnce(() -> {
          leds.breathLeft(255, 0, 0, 1.0);
          leds.breathRight(255, 0, 0, 1.0);
        }, leds).ignoringDisable(true)
    );

    // Teleop：左右彩虹
    RobotModeTriggers.teleop().onTrue(
        Commands.runOnce(leds::rainbowAll, leds)
    );

    // Auto：左右蓝色常亮
    RobotModeTriggers.autonomous().onTrue(
        Commands.runOnce(() -> {
          leds.solidLeft(0, 0, 255);
          leds.solidRight(0, 0, 255);
        }, leds)
    );
  }
}
