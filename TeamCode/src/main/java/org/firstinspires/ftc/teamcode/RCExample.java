package org.firstinspires.ftc.teamcode;

public class RCExample extends RobCommand {
    public static final int CMD_HOLD = 0;
    public static final int CMD_RELEASE = 1;
    public boolean skipWait = false;
    private Hardware hardware = null;

    private double startTime = 0.0;
    private int servoCMD;

    public RCExample(Hardware hardware, int servoCMD, boolean skipWait) {
        this.hardware = hardware;
        this.skipWait = skipWait;
    }

    public void run() {
        startTime = hardware.getCurrentTime();
        switch (servoCMD) {
            case CMD_HOLD:
//                hardware.pixelCabin.holdPixel();
                hardware.logMessage(false, "RCExample", "Claw Set To open Position");
                break;
            case CMD_RELEASE:
//                hardware.pixelCabin.releasePixel();
                hardware.logMessage(false, "RCExample", "Claw Set To grip Position");
                break;
        }
    }

    public boolean isComplete() {
        if (skipWait) {
            hardware.logMessage(false, "RCExample", "Command Complete, skipped wait");
            return true;
        }
        if ((hardware.getCurrentTime() - startTime) > 0.35) //XXX | change number later on, just random number for now
        {
            hardware.logMessage(false, "RCExample", "Command Complete, after wait");
            return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return "RCExample{" +
                ", servoCMD=" + servoCMD +
                ", skipWait=" + skipWait +
                ", startTime=" + startTime +
                '}';
    }
}