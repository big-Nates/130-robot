package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import androidx.annotation.NonNull;

public class RobCommand {
    Hardware hardware = null;
    public boolean hasCompleted = false;

    public RobCommand(){

    }
    public RobCommand(Hardware hardware){
       this.hardware = hardware;
    }

    public void run(){

    }

    public Action getAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return !hasCompleted;
            }
        };
    }
    public boolean isComplete(){
        return !hasCompleted;
    }
}
