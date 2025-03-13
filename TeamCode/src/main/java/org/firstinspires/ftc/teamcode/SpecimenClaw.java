package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class SpecimenClaw {
    private OpMode opMode = null;
    private Hardware hardware = null;

    private Servo specimenClawServo = null;

    public static final double OPEN_POS = .560; // NEED TO BE SET
    public static final double CLOSE_POS = .452; // NEED TO BE SET
    private boolean isOpen = true;

    public SpecimenClaw(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init(){
        specimenClawServo = hardware.specimenServo;
        closeClaw();
    }

    public void openClaw(){
        specimenClawServo.setPosition(OPEN_POS);
        isOpen = true;
    }

    public void closeClaw(){
        specimenClawServo.setPosition(CLOSE_POS);
        isOpen = false;
    }

    public void toggleClaw(){
        if(isOpen){
            closeClaw();
        }else{
            openClaw();
        }
    }

    public String getStateString(){
        if (isOpen) {
            return "Open";
        } else {
            return "Close";
        }
    }

    public boolean getState(){
        return isOpen;
    }

    public void setClawPosition(double position){
        specimenClawServo.setPosition(Math.min(Math.max(position, 0.0), 1.0));
    }
}
