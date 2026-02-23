package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Pose2d;

public final class Constants {
    public static final boolean LEDUsing = false;
    public class Shooter {
        public static final double conveyorSpeedkP = 10;
        public static final Slot0Configs flyWheelSlot0Configs = new Slot0Configs()
            .withKP(0.0).withKI(0).withKD(0)
            .withKV(0.1).withKA(0.0).withKS(0.15);

        public static final Slot0Configs conveyorSlot0Configs = new Slot0Configs()
            .withKP(0.0).withKI(0).withKD(0)
            .withKV(0.02).withKA(0.15).withKS(0);

        public static final Slot0Configs backboardSlot0Configs = new Slot0Configs()
            .withKP(0.01).withKI(0).withKD(0)
            .withKV(0.008).withKA(0.0).withKS(0.010);

        public class backboardPositionPID {
            public static final double kP = 0.1;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            
        }
        public static final double backboardSpeedMax = 60; //也就是output
        // 背板的上下限位角度
        public static final double backboardUpLimit = 3000.0;  // 背板上限角度（根据实际需求设置）
        public static final double backboardDownLimit = 0.0;  // 背板下限角度（根据实际需求设置）
    }
    public class ShooterCalculation{
        public static final double positionNear = 0;  // 近距离背板角度
        public static final double positionMid  = 1600;  // 中距离背板角度
        public static final double postionFar  = 3200;  // 远距离背板角度
    }
    public class AutoRotation {
        public static final boolean autoRotationToHubEnabled = true;
        public static final double autoRotationToHubkP = 0.12;
        public static final boolean autoRotationForBump = false;
        public static final double autoRotationForBumpkP = 0.2;
        public static final double autoRotationForBumpTargetDegrees = 45;

        public static final boolean teleopOffsetEnabled = true;
        public static final double teleopOffsetkP = 15;
    }

    public class StartingPoints{
        public class Red{
            public static final Pose2d Point1 = new Pose2d(12.93, 7.47, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
            public static final Pose2d Point2 = new Pose2d(14.314, 4.15, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-180));
        }
        
    }

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        SYSID,
        OTHER
    }
    public class Pigeon {
        public static final int CANID = 0;
    }

    public class Limelight {
        public static final String LIMELIGHT_NAME = "limelight-zy";
    }

    public class Field {
        public static final double RedHubPositionX = 11.914;
        public static final double RedHubPositionY = 4.034;
        
    }

    public class Intaker {

    public static final double intakeRotaterUpLimit = 0.0;  // rotator上限角度
    public static final double intakeRotaterDownLimit = 0.0;  // rotator下限角度

    public static final Slot0Configs intakeGetterSlot0Configs = new Slot0Configs()
        .withKP(0.00).withKI(0.00).withKD(0.00)
        .withKV(0.00).withKA(0.00).withKS(0.00);
    // getter 如果你只用固定速度开关，也可以先不启闭环；若用闭环速度，这里 kP/kV/KS 等需要实测调参(这句fromAI)

    public static final Slot0Configs intakeRotaterSlot0Configs = new Slot0Configs()
        .withKP(0.00).withKI(0.00).withKD(0.00)
        .withKV(0.00).withKA(0.00).withKS(0.00);
    //如果 rotater 受重力影响明显，可能需要用 withKG/withGravityType/withGravityArmPositionOffset（看机构形式）(这句fromAI)

    public static final double IntakeGetterSpeed = 0.0;
    // 单位RPS
    public static final double positionToleranceRotations = 0.00;
    //rotater容差
    }   
    
}