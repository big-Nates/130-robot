package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;

public class RCLiftRotate extends RobCommand {
    private Hardware hardware = null;
    private int position = 0;
    private double power = 0.0;
    private boolean skipWait = false;

    public Action action;

    public static double inchesToPositionConversion = .01185;

    public RCLiftRotate(Hardware hardware, int position, double power) {
        this.hardware = hardware;
        this.position = position;
        this.power = power;
        action = hardware.liftRotate.setPositionAction(this.position, this.power);
    }

    public RCLiftRotate(Hardware hardware, int position, double power, boolean skipWait) {
        this.hardware = hardware;
        this.position = position;
        this.power = power;
        this.skipWait = skipWait;
        action = hardware.liftRotate.setPositionAction(this.position, this.power);
    }

    public void run() {
        hardware.logMessage(false, "RCLiftRotate", "Command Ran, set to position " + position);
        action.run();
    }

    public boolean isComplete() {
        if (skipWait) {
            hardware.logMessage(false, "RCLiftRotate", "Command Complete, Skip Wait");
            return true;
        }

        return !action.run(hardware.packet);
    }

    public Action getAction(){
        return action;
    }

    @Override
    public String toString() {
        return "RCLiftRotate{" +
                ", position=" + position +
                ", power=" + power +
                ", skipWait=" + skipWait +
                '}';
    }
}
