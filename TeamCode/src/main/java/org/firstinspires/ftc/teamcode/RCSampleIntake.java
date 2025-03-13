package org.firstinspires.ftc.teamcode;

public class RCSampleIntake extends RobCommand {

    public static final int CMD_OFF = 0;
    public static final int CMD_ON = 1;

    public static final int CMD_TOGGLE_POWER = 2;

    public static final int CMD_CUSTOM_POWER = -1;

    public static boolean isIntakeDirection = true;
    private Hardware hardware = null;
    private double power = 0;
    private int intakeCMD = -1;
    private boolean skipWait = false;

    private final double OUTTAKE_DELAY_TIME = 0.5;

    private double delayTime = 0.0;
    private double startTime = 0.0;

    public RCSampleIntake(Hardware hardware, int intakeCMD, boolean isIntakeDirection, boolean skipWait) {
        this.hardware = hardware;
        this.intakeCMD = intakeCMD;
        this.isIntakeDirection = isIntakeDirection;
        if(isIntakeDirection){
            delayTime = 0.0;
        }else{
            delayTime = OUTTAKE_DELAY_TIME;
        }
        this.skipWait = skipWait;
    }

    public RCSampleIntake(Hardware hardware, double power, boolean skipWait) {
        this.hardware = hardware;
        this.power = power;
        this.skipWait = skipWait;
    }

    public void run() {
        startTime = hardware.getCurrentTime();
        switch (intakeCMD) {
            case CMD_OFF:
                hardware.sampleIntake.noRotation();
                hardware.logMessage(false, "RCSampleIntake", "Intake Set To off");
                break;
            case CMD_ON:
                if(isIntakeDirection){
                    hardware.sampleIntake.intakeRotation();
                    hardware.logMessage(false, "RCSampleIntake", "Intake Set To intaking");
                    break;
                }else{
                    hardware.sampleIntake.outtakeRotation();
                    hardware.logMessage(false, "RCSampleIntake", "Intake Set To outtaking");
                    break;
                }
            case CMD_TOGGLE_POWER:
                switch(hardware.sampleIntake.getState()){
                    case SampleIntake.INTAKING:
                    case SampleIntake.OUTTAKING:
                        if(isIntakeDirection){
                            hardware.sampleIntake.toggleDirection("Intake");
                            if(hardware.sampleIntake.getState() == SampleIntake.OUTTAKING){
                                delayTime = OUTTAKE_DELAY_TIME;
                            }else{
                                delayTime = 0.0;
                            }
                        }else{
                            hardware.sampleIntake.toggleDirection("Outtake");
                            if(hardware.sampleIntake.getState() == SampleIntake.OUTTAKING){
                                delayTime = OUTTAKE_DELAY_TIME;
                            }else{
                                delayTime = 0.0;
                            }
                        }
                        break;
                    case SampleIntake.STATIONARY:
                        if(isIntakeDirection){
                            hardware.sampleIntake.intakeRotation();
                            delayTime = 0.0;
                        }else{
                            hardware.sampleIntake.outtakeRotation();
                            delayTime = OUTTAKE_DELAY_TIME;
                        }
                        break;
                }
                break;
            case CMD_CUSTOM_POWER:
                hardware.sampleIntake.setPower(power);
                delayTime = 0.0;
        }
    }

    public boolean isComplete() {
        if (skipWait) {
            hardware.logMessage(false, "RCSampleIntake", "Command Complete, skipped wait");
            return true;
        }
        if ((hardware.getCurrentTime() - startTime) > delayTime) //XXX | change number later on, just random number for now
        {
            hardware.logMessage(false, "RCSampleIntake", "Command Complete, after wait");
            return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return "RCSampleIntake{" +
                ", servoCMD=" + intakeCMD +
                ", skipWait=" + skipWait +
                ", startTime=" + startTime +
                '}';
    }
}