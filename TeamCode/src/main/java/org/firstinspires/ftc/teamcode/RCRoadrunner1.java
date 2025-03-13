package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;

public class RCRoadrunner1 extends RobCommand {
    private Hardware hardware = null;
    private Pose2d previousPose = null;
    private Action action = null;
    private MecanumDrive drive = null;
    
    
    public RCRoadrunner(Action action){
        this.hardware = hardware;
        this.action = action;
    }


    public Action getAction(){
        return action;
    }
}
