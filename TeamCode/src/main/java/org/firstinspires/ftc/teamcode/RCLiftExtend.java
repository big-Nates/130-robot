package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;

public class RCLiftExtend extends RobCommand {
    private Hardware hardware = null;
    private int position = 50;
    private double power = 0.0;
    private boolean skipWait = false;
    private Action action;

    public static double inchesToPositionConversion = .01185;

    public RCLiftExtend(Hardware hardware, int position, double power) {
        this.hardware = hardware;
        this.position = position;
        this.power = power;
        action = hardware.linearLift.setPositionAction(this.position, this.power);
    }

    public RCLiftExtend(Hardware hardware, int position, double power, boolean skipWait) {
        this.hardware = hardware;
        this.position = position;
        this.power = power;
        this.skipWait = skipWait;
        action = hardware.linearLift.setPositionAction(this.position, this.power);
    }

    public void run() {
        hardware.logMessage(false, "RCLiftExtend", "Command Ran, set to position " + position);
        action.run();
    }

    public boolean isComplete() {
        if (skipWait) {
            hardware.logMessage(false, "RCLiftExtend", "Command Complete, Skip Wait");
            return true;
        }

        return !action.run(hardware.packet);
    }

    public Action getAction(){
        return action;
    }

    @Override
    public String toString() {
        return "RCLiftExtend{" +
                ", position=" + position +
                ", power=" + power +
                ", skipWait=" + skipWait +
                '}';
    }
}
