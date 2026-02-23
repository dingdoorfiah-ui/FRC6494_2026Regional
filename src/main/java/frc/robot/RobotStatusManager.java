package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotStatus;

public class RobotStatusManager {
    private RobotStatus robotStatus;
    
    public RobotStatusManager(){
        this.robotStatus = RobotStatus.AllTelop;
    }

    public RobotStatus getStatus(){
        return this.robotStatus;
    }

    public void setStatus(RobotStatus rs){
        this.robotStatus = rs;
    }

    public Command setStatusCommand(RobotStatus rs){
        return Commands.runOnce(()->setStatus(rs));
    }
}
