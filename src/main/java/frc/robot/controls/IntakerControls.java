package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakerSubsystem;

public class IntakerControls {
    private final IntakerSubsystem intakerSubsystem;
    private final CommandXboxController controller;

    private static final double ROTATER_STEP_PER_CYCLE = 0.00; // 需要你按实际速度调：越大越快（fromAI）

    public IntakerControls(IntakerSubsystem intakerSubsystem, CommandXboxController controller) {
        this.intakerSubsystem = intakerSubsystem;
        this.controller = controller;
    }

    public void configureBindings() {

        //B.A：按住下降 + getter转；松开后 rotater保持当前位置，getter继续转
        controller.a()
            .whileTrue(
                Commands.run(() -> {
                    // rotater下降：范围会被 subsystem 限制
                    double cur = intakerSubsystem.getIntakerotaterPosition();
                    intakerSubsystem.setIntakerRotaterPosition(cur - ROTATER_STEP_PER_CYCLE);

                    // getter 开
                    intakerSubsystem.setIntakeGetterOn(true);
                }, intakerSubsystem)
            )
            .onFalse(
                Commands.runOnce(() -> {
                    // 松：把当前位置作为target保持
                    double cur = intakerSubsystem.getIntakerotaterPosition();
                    intakerSubsystem.setIntakerRotaterPosition(cur);

                    // gtter继续转
                    intakerSubsystem.setIntakeGetterOn(true);
                }, intakerSubsystem)
            );

        // === B.B：按住上升+getter停；松开后rotater保持当前位置
        controller.b()
            .whileTrue(
                Commands.run(() -> {
                    // rotater升
                    double cur = intakerSubsystem.getIntakerotaterPosition();
                    intakerSubsystem.setIntakerRotaterPosition(cur + ROTATER_STEP_PER_CYCLE);

                    // getter关
                    intakerSubsystem.setIntakeGetterOn(false);
                }, intakerSubsystem)
            )
            .onFalse(
                Commands.runOnce(() -> {
                    // 松：保持当前位置
                    double cur = intakerSubsystem.getIntakerotaterPosition();
                    intakerSubsystem.setIntakerRotaterPosition(cur);

                    // getter仍然停止
                    intakerSubsystem.setIntakeGetterOn(false);
                }, intakerSubsystem)
            );
    }

    public Command defaultIntakerCommand() {
        return Commands.none();
    }
}
