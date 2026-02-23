package frc.robot.tuning;
import java.util.function.Function;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.*;

public class ConfigTalonFXMotorTuner {
  private final TalonFX motor;
  private final String motorName;
  private final Slot0Configs motorSlot0Configs;
  private final NetworkTable table;

  // ---- Subscribers：读取 Elastic 改过的新值 ----
  private final BooleanSubscriber enableSub;
  private final DoubleSubscriber KpSub;
  private final DoubleSubscriber KiSub;
  private final DoubleSubscriber KdSub;
  private final DoubleSubscriber KvSub;
  private final DoubleSubscriber KaSub;
  private final DoubleSubscriber KsSub;

  private final BooleanSubscriber enableRuningSub;
  private final DoubleSubscriber runningSpeedRPS;
  private double lastKp = 0, lastKi = 0, lastKd = 0, lastKv = 0, lastKa = 0, lastKs = 0;
  
  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);
  public ConfigTalonFXMotorTuner(TalonFX motor_, String motorName_, Slot0Configs motorSlot0Configs_) {
    this.motor = motor_;
    this.motorName = motorName_;
    this.motorSlot0Configs = motorSlot0Configs_;

    this.table = NetworkTableInstance.getDefault().getTable("Tuning/" + motorName);

    table.getBooleanTopic(motorName+"Enable").publish().set(false);
    table.getDoubleTopic(motorName+"kP").publish().set(motorSlot0Configs.kP);;
    table.getDoubleTopic(motorName+"kI").publish().set(motorSlot0Configs.kI);
    table.getDoubleTopic(motorName+"kD").publish().set(motorSlot0Configs.kD);
    table.getDoubleTopic(motorName+"kV").publish().set(motorSlot0Configs.kV);
    table.getDoubleTopic(motorName+"kA").publish().set(motorSlot0Configs.kA);
    table.getDoubleTopic(motorName+"kS").publish().set(motorSlot0Configs.kS);

    table.getBooleanTopic(motorName+"EnableRunning").publish().set(false);;
    table.getDoubleTopic(motorName+"RunningSpeedRPS").publish().set(0);;
    
    this.enableSub = table.getBooleanTopic(motorName+"Enable").subscribe(false);
    this.KpSub = table.getDoubleTopic(motorName+"kP").subscribe(motorSlot0Configs.kP);
    this.KiSub = table.getDoubleTopic(motorName+"kI").subscribe(motorSlot0Configs.kI);
    this.KdSub = table.getDoubleTopic(motorName+"kD").subscribe(motorSlot0Configs.kD);
    this.KvSub = table.getDoubleTopic(motorName+"kV").subscribe(motorSlot0Configs.kV);
    this.KaSub = table.getDoubleTopic(motorName+"kA").subscribe(motorSlot0Configs.kA);
    this.KsSub = table.getDoubleTopic(motorName+"kS").subscribe(motorSlot0Configs.kS);
    this.enableRuningSub = table.getBooleanTopic(motorName+"EnableRunning").subscribe(false);
    this.runningSpeedRPS = table.getDoubleTopic(motorName+"RunningSpeedRPS").subscribe(0);
  }

  public void periodic(Runnable func) {
    if (!enableSub.get()) return;
    double kp = KpSub.get();
    double ki = KiSub.get();
    double kd = KdSub.get();
    double kv = KvSub.get();
    double ka = KaSub.get();
    double ks = KsSub.get();

    if (!(kp == lastKp && ki == lastKi && kd == lastKd && kv == lastKv && ka == lastKa && ks == lastKs)) {

      Slot0Configs slot0 = new Slot0Configs()
          .withKP(kp).withKI(ki).withKD(kd).withKV(kv).withKA(ka).withKS(ks);
        
      motor.getConfigurator().apply(slot0);
      System.out.println("applyed settings: "+motorName);

      lastKp = kp; lastKi = ki; lastKd = kd; lastKv = kv; lastKa = ka; lastKs = ks;

      if(func!=null){
        func.run();
      }
    }
    
    if (enableRuningSub.get()) {;
      double speed = runningSpeedRPS.get();
      motor.setControl(velocityRequest.withVelocity(speed));
    // System.out.println("running motor with speed: "+speed+" on motor: "+motorName);
    }else{
      motor.setControl(velocityRequest.withVelocity(0));
    }
  }

  
}
