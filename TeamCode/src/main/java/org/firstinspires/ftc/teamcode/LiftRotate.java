package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import androidx.annotation.NonNull;


public class LiftRotate {
    private OpMode opMode;
    private Hardware hardware;
    private DcMotorEx rotateMotor;

    private TouchSensor home;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime timeout = new ElapsedTime();

    private final double liftPower = 1.0;
    private final double liftHomingPower = -0.35;
    private double startTime = 0;
    private int previousTargetPos = 0;
    private int liftRotateCurrentPosition = -999;
    private final int liftRotateSafePosition = 850;
    public static final int LIFT_VERT_POS = 2677;
    public static final int LIFT_HORI_POS = 100;
    public static final int LIFT_MAX_POS = 2820; // NEED TO BE SET
    public static final int LIFT_MIN_POS = 0; // NEED TO BE SET
    public static final int LIFT_RETRACT_POS = 50;
    public static final int LIFT_SUB_POS = 400;
    public static final int LIFT_STOW_POS = 0;
    public static final int LIFT_LOW_BASKET = 2091;
    public static final int LIFT_HIGH_BASKET = 2430;
    public static final int LIFT_CHAMBER = 2610;

    //TODO: Find Real Max Rate of LinearLift & RotateLift
    public static final double MAX_ROTATE_TIME = 1.66;
    public static final double LIFT_ROTATE_MAX_SPEED = LIFT_MAX_POS /MAX_ROTATE_TIME;

    private static final double MAX_TIMEOUT = 5.0;

    private static final int LIFTNOTHOMED = 0;
    private static final int LIFTFINDINGHOME = 1;
    private static final int LIFTBACKOFFHOME = 2;
    private static final int LIFTREADY = 3;
    private int state = LIFTNOTHOMED;
    private boolean isRising = false;



    public LiftRotate(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init() {
        opMode.telemetry.addData("Lift Rotate Status", "Initializing");
        rotateMotor = hardware.rotateMotor;
        rotateMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        home = hardware.liftRotateHome;
        runtime.reset();
        timeout.reset();

        opMode.telemetry.addData("Lift Rotate Status", "Initialized");
        opMode.telemetry.update();
    }


    public void doLoop(){
        opMode.telemetry.addData("Lift Rotate Status", "Starting. Finding home...");
//        opMode.telemetry.update();
        switch (state) {
            case LIFTNOTHOMED:
                break;

            case LIFTBACKOFFHOME:
                if (opMode.time - startTime >= 0.7) {
                    findHome();
                }
                break;

            case LIFTFINDINGHOME:
                hardware.logMessage(false, "Life Rotate", "lift is in finding home state");
                if (!home.isPressed()) {
                    if (opMode.time - startTime >= MAX_TIMEOUT) {
                        rotateMotor.setPower(0);
                        opMode.telemetry.addLine("Lift Rotate could not find home position");
                        hardware.logMessage(true, "Lift Rotate", "COULD NOT LOCATE HOME POSITION");
                    }
                    break;
                } else {
                    rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    setPosition(0, liftPower);
                    hardware.logMessage(false, "Lift Rotate", "Lift State:  Ready");
                    state = LIFTREADY;
                }
                break;
            }
        liftRotateCurrentPosition = rotateMotor.getCurrentPosition();

    }

    public double getCurrentPow() {
        return rotateMotor.getPower();
    }

    public int getCurrentPos() {
        return rotateMotor.getCurrentPosition();
    }

    //Lift Movement
    public void setPosition(int targetPos) {
        this.setPosition(targetPos, liftPower);
    }

    public void setPosition(int targetPos, double targetPow) {
        int tp = Math.max(Math.min(targetPos, LIFT_VERT_POS), LIFT_MIN_POS);
        isRising = ((tp <= previousTargetPos));
        rotateMotor.setTargetPosition(tp);
        rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double tpw = targetPow;
        rotateMotor.setPower(tpw);
        previousTargetPos = tp;
    }

    public void goMin(){setPosition(LIFT_MIN_POS);}
    public void goMax() {
        setPosition(LIFT_MAX_POS);
    }
    public void goStow(){
        setPosition(LIFT_STOW_POS);
    }
    public void goLowBasketAng(){
        setPosition(LIFT_LOW_BASKET);
    }

    public void goHighBasketAng(){
        setPosition(LIFT_HIGH_BASKET);
    }
    public void goChamberAng(){
        setPosition(LIFT_CHAMBER);
    }

    public void goSubAng(){
        setPosition(LIFT_SUB_POS);
    }

    public void calibrateLift() {
        hardware.logMessage(false, "Lift Rotate", "Starting to calibrate Lift");
        state = LIFTNOTHOMED;
        if (home.isPressed()) {
            backOffHome();
        } else {
            findHome();
        }
    }

    public void findHome() {
        startTime = opMode.time;
        hardware.rotateMotor.setMotorEnable();
        rotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotateMotor.setPower(liftHomingPower);
        hardware.logMessage(false, "Lift Rotate", "Lift Rotate State: Finding Home");
        state = LIFTFINDINGHOME;
    }

    public void backOffHome() {
        startTime = opMode.time;
        hardware.rotateMotor.setMotorEnable();
        rotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotateMotor.setPower(liftPower * 0.4);

        hardware.logMessage(false, "Lift Rotate", "Lift Rotate State: Backing Off Home");
        state = LIFTBACKOFFHOME;
    }

    public Action setPositionAction(int targetPos, double targetPower){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setPosition(targetPos, targetPower);
                if (Math.abs(hardware.linearLift.getCurrentPos() - targetPos) < 20) {
                    hardware.logMessage(false, "RCLiftRotate", "Command Complete, at requested position");
                    return true;
                }
                return false;
            }

        };
    }

    public void stop() {
        rotateMotor.setPower(0.0);
        rotateMotor.setVelocity(0.0);
        rotateMotor.setMotorDisable();
    }

    public int manualPositionIncrement(double power, int liftPreviousRotatePosition){
        return liftPreviousRotatePosition + (int) (power * LiftRotate.LIFT_ROTATE_MAX_SPEED * hardware.getDeltaTime());
    }

}
