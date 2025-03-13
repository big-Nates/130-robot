package org.firstinspires.ftc.teamcode;

public class RCLiftCalibrate extends RobCommand {
    private Hardware hardware = null;
    private int position = 50;
    private double power = 0.0;
    private boolean skipWait = false;

    public static double inchesToPositionConversion = .01185;

    public RCLiftCalibrate(Hardware hardware) {
        this.hardware = hardware;
    }

    public RCLiftCalibrate(Hardware hardware, boolean skipWait) {
        this.hardware = hardware;
        this.skipWait = skipWait;
    }

    public void run() {
        hardware.logMessage(false, "RCLiftCalibrate", "Command Ran, calibrating lift");
        hardware.linearLift.calibrateLift();
    }

    public boolean isComplete() {
        if (skipWait) {
            hardware.logMessage(false, "RCLiftCalibrate", "Command Complete, Skip Wait");
            return true;
        }
        if (hardware.linearLift.getState() == LinearLift.LINEARLIFTREADY) {
            hardware.logMessage(false, "RCLiftCalibrate", "Command Complete, successfully calibrated");
            return true;
        }
        return false;
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
