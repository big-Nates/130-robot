package org.firstinspires.ftc.teamcode;

public class RCSpecimenClaw extends RobCommand {
    public static final int CMD_CLOSE = 0;
    public static final int CMD_OPEN = 1;
    private Hardware hardware = null;
    private int servoCMD = 0;
    private boolean skipWait = false;

    private double startTime = 0.0;

    public RCSpecimenClaw(Hardware hardware, int servoCMD, boolean skipWait) {
        this.hardware = hardware;
        this.servoCMD = servoCMD;
        this.skipWait = skipWait;
    }

    public void run() {
        startTime = hardware.getCurrentTime();
        switch (servoCMD) {
            case CMD_CLOSE:
                hardware.specimenClaw.closeClaw();
                hardware.logMessage(false, "RCSpecimenClaw", "Specimen Claw Set To close Position");
                break;
            case CMD_OPEN:
                hardware.specimenClaw.openClaw();
                hardware.logMessage(false, "RCSpecimenClaw", "Specimen Claw Set To open Position");
                break;
        }
    }

    public boolean isComplete() {
        if (skipWait) {
            hardware.logMessage(false, "RCSpecimenClaw", "Command Complete, skipped wait");
            return true;
        }
        if ((hardware.getCurrentTime() - startTime) > 0.35) //XXX | change number later on, just random number for now
        {
            hardware.logMessage(false, "RCSpecimenClaw", "Command Complete, after wait");
            return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return "RCSpecimenClaw{" +
                ", servoCMD (to open)=" + servoCMD +
                ", skipWait=" + skipWait +
                ", startTime=" + startTime +
                '}';
    }
}