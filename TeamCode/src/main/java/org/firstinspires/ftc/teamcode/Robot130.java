package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Robot130 {
    private OpMode opMode = null;
    private Hardware hardware = null;

    //TIMING
//    private ElapsedTime runtime = new ElapsedTime();
//    private ElapsedTime timeout = new ElapsedTime();

    //MAIN STATE MACHINE VARIABLES
    private final int NOT_READY = 0;
    private final int READY = 1;
    public int loopState = NOT_READY;

    //DROP STATES
    private final int DROP_CONE_LIFT_MOVING_DOWN = 10;
    private final int DROP_CONE_DROPPING = 11;
    private final int DROP_CONE_LIFT_MOVING_UP = 12;

    String[] loopListValues = {"NOT_READY", "READY", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "NULL", "DROP_CONE_LIFT_MOVING_DOWN", "DROP_CONE_DROPPING", "DROP_CONE_LIFT_MOVING_UP"};

    public RobotCommandStack robotCommandStack = null;
    public RobotCommandStack roadrunnerCommandStack = null;

    public RobotCommandStack thirdStack = null;
    public RobotCommandStack fourthStack = null;





    //OTHER
    private final int dropPos = 0;
    private final int preDropPos = 0;
    private final double liftTimeout = 0.0;
    private final double clawOpenTime = 0.0;


    public Robot130(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
        this.robotCommandStack = new RobotCommandStack(hardware, "RobotCommandStack");
        this.roadrunnerCommandStack = new RobotCommandStack(hardware, "RoadrunnerCommandStack");
        this.thirdStack = new RobotCommandStack(hardware, "ThirdCommandStack");
        this.fourthStack = new RobotCommandStack(hardware, "FourthCommandStack");
    }

    public void init() {
//        runtime.reset();
//        timeout.reset();
    }
//    public void doInitLoop() {
//        switch (loopState) {
//            case NOT_READY:
//                break;
//
//            case READY:
//                break;
//
//            case DROP_CONE_LIFT_MOVING_DOWN:
//                if (Math.abs(hardware.lift.getCurrentPos() - dropPos) < 10) {
//                    hardware.claw.open();
//                    clawOpenTime = opMode.time;
//                    loopState = DROP_CONE_DROPPING;
//                } else if (opMode.time - liftTimeout > 2.0) {
//                    opMode.telemetry.addLine("Lift Timed Out going to position: " + dropPos + "currently at" + hardware.lift.getCurrentPos());
//                    hardware.logMessage(true, "Lift", "Lift Timed Out getting to position" + dropPos + "current position is" + hardware.lift.getCurrentPos());
//                }
//                break;
//
//            case DROP_CONE_DROPPING:
//                if (opMode.time - clawOpenTime >= 0.5) {
//                    hardware.lift.setPosition(preDropPos);
//                    liftTimeout = opMode.time;
//                    loopState = DROP_CONE_LIFT_MOVING_UP;
//                }
//                break;
//
//            case DROP_CONE_LIFT_MOVING_UP:
//                if (Math.abs(hardware.lift.getCurrentPos() - preDropPos) < 10) {
//                    loopState = READY;
//                } else if (opMode.time - liftTimeout > 2.0) {
//                    opMode.telemetry.addLine("Lift Timed Out going to position " + preDropPos + "currently at" + hardware.lift.getCurrentPos());
//                    hardware.logMessage(true, "Lift", "Lift Timed Out getting to position" + preDropPos + "current position is" + hardware.lift.getCurrentPos());
//                }
//                break;
//        }
//        opMode.telemetry.addLine("Robot State: " + loopListValues[loopState]);
//        opMode.telemetry.update();
//    }
//    public void doLoop() {
//        switch (loopState) {
//            case NOT_READY:
//                break;
//
//            case READY:
//                break;
//
//            case DROP_CONE_LIFT_MOVING_DOWN:
//                if (Math.abs(hardware.lift.getCurrentPos() - dropPos) < 10) {
//                    hardware.claw.open();
//                    clawOpenTime = opMode.time;
//                    loopState = DROP_CONE_DROPPING;
//                } else if (opMode.time - liftTimeout > 2.0) {
//                    opMode.telemetry.addLine("Lift Timed Out going to position: " + dropPos + "currently at" + hardware.lift.getCurrentPos());
//                    hardware.logMessage(true, "Lift", "Lift Timed Out getting to position" + dropPos + "current position is" + hardware.lift.getCurrentPos());
//                }
//                break;
//
//            case DROP_CONE_DROPPING:
//                if (opMode.time - clawOpenTime >= 0.5) {
//                    hardware.lift.setPosition(preDropPos);
//                    liftTimeout = opMode.time;
//                    loopState = DROP_CONE_LIFT_MOVING_UP;
//                }
//                break;
//
//            case DROP_CONE_LIFT_MOVING_UP:
//                if (Math.abs(hardware.lift.getCurrentPos() - preDropPos) < 10) {
//                    loopState = READY;
//                } else if (opMode.time - liftTimeout > 2.0) {
//                    opMode.telemetry.addLine("Lift Timed Out going to position " + preDropPos + "currently at" + hardware.lift.getCurrentPos());
//                    hardware.logMessage(true, "Lift", "Lift Timed Out getting to position" + preDropPos + "current position is" + hardware.lift.getCurrentPos());
//                }
//                break;
//        }
//        opMode.telemetry.addLine("Robot State: " + loopListValues[loopState]);
//        opMode.telemetry.update();
//    }
//
//    public void dropCone() {
//        preDropPos = hardware.lift.getCurrentPos();
//        dropPos = preDropPos - 250;
//        hardware.lift.setPosition(dropPos);
//        liftTimeout = opMode.time;
//        loopState = DROP_CONE_LIFT_MOVING_DOWN;
//    }


    public void processCommands(){
        robotCommandStack.processCommands();
        roadrunnerCommandStack.processCommands();
        thirdStack.processCommands();
        fourthStack.processCommands();
    }
//    public boolean isRoadrunnerActive(){
//        if(robotCommandStack.isRoadRunnerActive()){
//            if(roadrunnerCommandStack.isRoadRunnerActive()){
//                if(thirdStack.isRoadRunnerActive()){
//                    if(fourthStack.isRoadRunnerActive()){
//                        return true;
//                    }
//                }
//            }
//        }
//        return false;
//   }



    public void addCommand(RobCommand command){
        robotCommandStack.addCommand(command);
    }

    public void addRoadrunnerCommand(RobCommand command){
        roadrunnerCommandStack.addCommand(command);
    }

    public void addToThirdStack(RobCommand command){
        thirdStack.addCommand(command);
    }

    public void addToFourthStack(RobCommand command){
        fourthStack.addCommand(command);
    }






}
