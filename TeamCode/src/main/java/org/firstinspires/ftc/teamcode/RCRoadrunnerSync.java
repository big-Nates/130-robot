package org.firstinspires.ftc.teamcode;

public class RCRoadrunnerSync extends RobCommand{
    public               double delayTime = 0;
    public               double endTime = 0;
    private RobCommand command = null;

    public RCRoadrunnerSync(Hardware hardware, RobCommand command){
        this.hardware = hardware;
        this.command = command;
    }

    public void run(){
        hardware.logMessage(false,"RCRoadrunnerSync","Sync Command Started");

    }

    public boolean isComplete(){
       if (command.hasCompleted){
           hardware.logMessage(false,"RCRoadrunnerSync","Sync Command Completed");
           return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return "RCWait{" +
                "delayTime=" + delayTime +
                ", endTime=" + endTime +
                '}';
    }
}