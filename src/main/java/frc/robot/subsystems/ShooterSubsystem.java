package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    // 定义飞轮、传动和背板电机
    public TalonFX flywheelMotorLeft;
    private TalonFX flywheelMotorRight;
    public TalonFX conveyorMotor;
    public TalonFXS backboardMotor;
    public Encoder backboardEncoder;

    // PID 控制器，控制背板角度
    public ProfiledPIDController backboardPID;
    private Follower flyWheelFollower;

    private NetworkTable shooterNetworkTable;

    private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

    public enum Mode{
        Inalyzed,
        UnInalyed,
        Inalyzing;
    }
    public Mode currentBackboardInalyzeMode;
    // 初始化各个部件
    public ShooterSubsystem() {
        // 初始化飞轮电机
        flywheelMotorLeft = new TalonFX(41);  // 41是飞轮left的CAN ID
        flywheelMotorRight = new TalonFX(42);  // 42是飞轮right的CAN ID
        flywheelMotorLeft.getConfigurator().apply(Constants.Shooter.flyWheelSlot0Configs);
        flywheelMotorRight.getConfigurator().apply(Constants.Shooter.flyWheelSlot0Configs);
        // 初始化传动电机
        conveyorMotor = new TalonFX(43);
        conveyorMotor.getConfigurator().apply(Constants.Shooter.conveyorSlot0Configs);

        var configs1 = new MotorOutputConfigs();
        configs1.Inverted = InvertedValue.Clockwise_Positive;
        flywheelMotorLeft.getConfigurator().apply(configs1);
        flyWheelFollower = new Follower(flywheelMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(50);
        
        var configs2 = new MotorOutputConfigs();
        configs2.Inverted = InvertedValue.Clockwise_Positive;
        conveyorMotor.getConfigurator().apply(configs2);
        
        // 初始化背板电机和编码器
        backboardMotor = new TalonFXS(44);  
        backboardMotor.getConfigurator().apply(Constants.Shooter.backboardSlot0Configs);
        backboardEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        backboardEncoder.setSamplesToAverage(5);
        backboardEncoder.setMinRate(1.0);
        backboardEncoder.setDistancePerPulse(1);
        
        // 初始化背板的PID控制器
        backboardPID = new ProfiledPIDController(Constants.Shooter.backboardPositionPID.kP, Constants.Shooter.backboardPositionPID.kI, Constants.Shooter.backboardPositionPID.kD,
                                                    new Constraints(10000,100000));
        
        backboardPID.setTolerance(10.0); 
        
        shooterNetworkTable = NetworkTableInstance.getDefault().getTable("Shooter");

        currentBackboardInalyzeMode = Mode.UnInalyed;
    }

    public void setRightFollowLeft(){
        flywheelMotorRight.setControl(flyWheelFollower);
    }

    public void applyLeftConfigurationToRight(){
        Slot0Configs configs = new Slot0Configs();
        flywheelMotorLeft.getConfigurator().refresh(configs);
        flywheelMotorRight.getConfigurator().apply(configs);
    }


    // 设置飞轮的速度
    public void setFlywheelSpeedByRPS(double targetSpeed) {
        flywheelMotorLeft.setControl(velocityRequest.withVelocity(targetSpeed)); //RPS 转每秒
        setRightFollowLeft();
        // System.out.println("set flywheel "+targetSpeed);
    }

    // 设置传动的速度
    public void setConveyorSpeedByRPS(double speed) {
        conveyorMotor.setControl(velocityRequest.withVelocity(speed));  // 设置传动电机速度
    }

    //调试用
    public void setBackboardSpeedByRPS(double speed){
        backboardMotor.setControl(velocityRequest.withVelocity(speed));
    }

    // 设置背板的目标位置，通过PID控制器来调整  范围0-3000
    public void setBackboardPosition(double targetPosition) {
        // 限制目标位置在上下限之间
        if (targetPosition > Constants.Shooter.backboardUpLimit) {
            targetPosition = Constants.Shooter.backboardUpLimit;  // 限制最大上限
        } else if (targetPosition < Constants.Shooter.backboardDownLimit) {
            targetPosition = Constants.Shooter.backboardDownLimit;  // 限制最小下限
        }
        backboardPID.setGoal(targetPosition);
    }
    public void outputBackboard(){

        // 使用PID控制器来计算背板的运动速度
        double pidOutput = backboardPID.calculate(getBackboardPosition());
        pidOutput = Math.max(-Constants.Shooter.backboardSpeedMax, Math.min(Constants.Shooter.backboardSpeedMax, pidOutput)); // 限制输出范围在-1到1之间
        shooterNetworkTable.getEntry("backboardPIDOutput").setDouble(pidOutput);
        // // 设置背板电机的速度，根据PID输出进行调整
        setBackboardSpeedByRPS(pidOutput);
    }
    // 停止所有电机
    public void stopMotors() {
        flywheelMotorLeft.set(0);
        flywheelMotorRight.set(0);
        conveyorMotor.set(0);
        backboardMotor.set(0);
    }

    // 获取背板当前角度
    public double getBackboardPosition() {
        return backboardEncoder.getDistance();  // 返回背板电机编码器的当前角度
    }

    // 检查背板是否到达目标角度
    public boolean isBackboardAtTarget() {
        return backboardPID.atGoal();  // 判断背板是否到达目标角度
    }

    @Override
    public void periodic() {
        if(currentBackboardInalyzeMode != Mode.Inalyzing){
            backboardPID.calculate(getBackboardPosition());
            if(isBackboardAtTarget()){
                backboardMotor.set(0);
            }else{
                outputBackboard();
            }
        }
        shooterNetworkTable.getEntry("flywheelSpeed").setDouble(flywheelMotorLeft.getVelocity().getValueAsDouble());
        shooterNetworkTable.getEntry("flywheelPIDReference").setDouble(flywheelMotorLeft.getClosedLoopReference().getValueAsDouble());
        shooterNetworkTable.getEntry("conveyorSpeed").setDouble(conveyorMotor.getVelocity().getValueAsDouble());
        shooterNetworkTable.getEntry("conveyorPIDReference").setDouble(conveyorMotor.getClosedLoopReference().getValueAsDouble());
        shooterNetworkTable.getEntry("backboardCurrentRate").setDouble(getBackboardPosition());
        shooterNetworkTable.getEntry("backboardTargetRate").setDouble(backboardPID.getSetpoint().position);
        shooterNetworkTable.getEntry("isBackboardAtTarget").setBoolean(isBackboardAtTarget());
        shooterNetworkTable.getEntry("distanceGetted").setDouble(getDistance());
        shooterNetworkTable.getEntry("InalyzeBackboardMode").setString(currentBackboardInalyzeMode.name());
    }

    public double getDistance(){
        //获得车子距离Hub的距离
        return NetworkTableInstance.getDefault().getTable("AutoControl").getEntry("distanceToHub").getDouble(0.0);
    }
}
