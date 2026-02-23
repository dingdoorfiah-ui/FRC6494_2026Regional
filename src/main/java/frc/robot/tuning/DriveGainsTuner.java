package frc.robot.tuning;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.networktables.*;

public class DriveGainsTuner {
  private final NetworkTable table =
      NetworkTableInstance.getDefault().getTable("Tuning/Drive");

  // ---- Publishers：让 Elastic 能“看到”这些键，并给初值 ----
  private final BooleanPublisher enablePub =
      table.getBooleanTopic("Enable").publish();
  
  private final DoublePublisher kpPub = table.getDoubleTopic("kP").publish();
  private final DoublePublisher kiPub = table.getDoubleTopic("kI").publish();
  private final DoublePublisher kdPub = table.getDoubleTopic("kD").publish();
  private final DoublePublisher kvPub = table.getDoubleTopic("kV").publish();

  // ---- Subscribers：读取 Elastic 改过的新值 ----
  private final BooleanSubscriber enableSub =
      table.getBooleanTopic("Enable").subscribe(false);

  private final DoubleSubscriber kpSub = table.getDoubleTopic("kP").subscribe(0.01);
  private final DoubleSubscriber kiSub = table.getDoubleTopic("kI").subscribe(0.0);
  private final DoubleSubscriber kdSub = table.getDoubleTopic("kD").subscribe(0.0);
  private final DoubleSubscriber kvSub = table.getDoubleTopic("kV").subscribe(0.124);

  private double lastKP = Double.NaN, lastKI = Double.NaN, lastKD = Double.NaN, lastKV = Double.NaN;

  private final TalonFX[] driveMotors;

  public DriveGainsTuner(SwerveDrivetrain<?, ?, ?> drivetrain) {
    // 发布初始值（这一步做完 Elastic 就能看到这些键）
    enablePub.set(false);
    kpPub.set(0.01);
    kiPub.set(0.0);
    kdPub.set(0.0);
    kvPub.set(0.124);

    var modules = drivetrain.getModules();
    driveMotors = new TalonFX[modules.length];
    for (int i = 0; i < modules.length; i++) {
      driveMotors[i] = (TalonFX) modules[i].getDriveMotor();
    }
  }

  public void periodic() {
    if (!enableSub.get()) return;

    double kp = kpSub.get();
    double ki = kiSub.get();
    double kd = kdSub.get();
    double kv = kvSub.get();

    if (kp == lastKP && ki == lastKI && kd == lastKD && kv == lastKV) return;

    Slot0Configs slot0 = new Slot0Configs()
        .withKP(kp).withKI(ki).withKD(kd).withKV(kv);

    for (TalonFX m : driveMotors) {
      m.getConfigurator().apply(slot0);
    }

    lastKP = kp; lastKI = ki; lastKD = kd; lastKV = kv;
  }
}
