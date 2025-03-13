package org.firstinspires.ftc.teamcode;

import android.app.Notification;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import androidx.annotation.NonNull;

public class LinearLift {

    private OpMode opMode;
    private Hardware hardware;
    private DcMotorEx linearLiftMotor;
    private TouchSensor home;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime timeout = new ElapsedTime();

    private final double upLiftPower = 1.0;
    private final double downLiftPower = 1.0;
    public static final double LIFT_MAX_SPEED = 1.0; // NEED TO BE SET

    public int rolling_max = 0;

    private final double liftHomingPower = -0.35;
    private double startTime = 0;
    private int previousTargetPos = 0;
    private int liftRotateCurrentPosition = -999;
    private final int liftRotateSafePosition = 850;
    public static final int LINEARLIFT_MAXPOS = 5800; // NEED TO BE SET
    public static final int LiNEARLIFT_MAX_HORI_POS = 2800;
    public static final int LINEAR_LIFT_MIN_POS = 50; // NEED TO BE SET
    public static final int LINEAR_LIFT_STOW_POS = 50;
    public static final int LINEAR_LIFT_WALL_POS = 0;
    public static final int LINEAR_LIFT_ABOVE_WALL = LINEAR_LIFT_WALL_POS + 580;
    public static final int LINEAR_LIFT_SUB_POS = 0;
    public static final int LINEAR_LIFT_LOW_BASKET = 2724;
    public static final int LINEAR_LIFT_HIGH_BASKET = 5000;
    public static final int LINEAR_LIFT_ABOVE_LOW_CHAMBER = 0;
    public static final int LINEAR_LIFT_LOW_CHAMBER = 0;
    public static final int LINEAR_LIFT_ABOVE_HIGH_CHAMBER = 2500;
    public static final int LINEAR_LIFT_HIGH_CHAMBER = 1920;
    public static final double MAX_EXTENSION_TIME = 3.41;

    public static final double LIFT_MANUAL_SPEED = LINEARLIFT_MAXPOS/MAX_EXTENSION_TIME;

    private static final double MAX_TIMEOUT = 5.0;

    public static final int LINEARLIFTNOTHOMED = 0;
    public static final int LINEARLIFTFINDINGHOME = 1;
    public static final int LINEARLIFTBACKOFFHOME = 2;
    public static final int LINEARLIFTREADY = 3;
    private int state = LINEARLIFTNOTHOMED;
    private boolean useMax = true;
    private boolean isRising = false;



    public LinearLift(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init() {
        opMode.telemetry.addData("Linear Lift Status", "Initializing");
        linearLiftMotor = hardware.linearMotor;
        linearLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        home = hardware.liftLinearHome;
        runtime.reset();
        timeout.reset();

        opMode.telemetry.addData("Linear Lift Status", "Initialized");
        opMode.telemetry.update();
    }


    public void doLoop(){
        opMode.telemetry.addData("Linear Lift Status", "Starting. Finding home...");
//        opMode.telemetry.update();
        switch (state) {
            case LINEARLIFTNOTHOMED:
                break;

            case LINEARLIFTBACKOFFHOME:
                if (opMode.time - startTime >= 0.2) {
                    findHome();
                }
                break;

            case LINEARLIFTFINDINGHOME:
                hardware.logMessage(false, "Linear Life", "lift is in finding home state");
                if (!home.isPressed()) {
                    if (opMode.time - startTime >= MAX_TIMEOUT) {
                        linearLiftMotor.setPower(0);
                        opMode.telemetry.addLine("Linear Lift could not find home position");
                        hardware.logMessage(true, "Linear Lift", "COULD NOT LOCATE HOME POSITION");
                    }
                    break;
                } else {
                    linearLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    setPosition(50, 0.7);
                    hardware.logMessage(false, "Linear Lift", "Linear Lift State:  Ready");
                    state = LINEARLIFTREADY;
                }
                break;
        }
        liftRotateCurrentPosition = linearLiftMotor.getCurrentPosition();
//        if (isRising && (liftCurrentPosition >= liftSafePosition)) {
//            hardware.pixelCabin.goToReleasePosition();
//        } else if ((liftCurrentPosition >= liftSafePosition) && hardware.liftManualMode) {
//            hardware.pixelCabin.goToReleasePosition();
//        } else {
//            hardware.pixelCabin.goToStowPosition();
//            hardware.pixelCabin.holdPixel();
//        }

//        if (!isRising) {
//            hardware.pixelCabin.goToStowPosition();
//            hardware.pixelCabin.holdPixel();
//        } else if (liftCurrentPosition >= liftSafePosition) {
//            hardware.pixelCabin.goToReleasePosition();
//        }

    }

    public double getCurrentPow() {
        return linearLiftMotor.getPower();
    }

    public int getCurrentPos() {
        return linearLiftMotor.getCurrentPosition();
    }

    //Lift Movement
    public void setPosition(int targetPos) {
        if(targetPos > getCurrentPos()){
            this.setPosition(targetPos, upLiftPower);
        }else{
            this.setPosition(targetPos, downLiftPower);
        }

    }

    public void setPosition(int targetPos, double targetPow) {
        int tp = 0;
        if(useMax){
            tp = Math.max(Math.min(targetPos, LINEARLIFT_MAXPOS), LINEAR_LIFT_MIN_POS);
        }else{
            tp = Math.max(Math.min(targetPos, LiNEARLIFT_MAX_HORI_POS), LINEAR_LIFT_MIN_POS);
        }
        isRising = ((tp <= previousTargetPos));

        linearLiftMotor.setTargetPosition(tp);
        linearLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double tpw = targetPow;
        linearLiftMotor.setPower(tpw);
        previousTargetPos = tp;
    }

    public void useMax(){
        useMax = true;
    }

    public void useHorizontalMax(){
        useMax = false;
    }

    public Action setPositionAction(int targetPos, double targetPower){
        return new Action() {
            
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                
                setPosition(targetPos, targetPower);
                if (Math.abs(hardware.linearLift.getCurrentPos() - targetPos) < 20) {
                    hardware.logMessage(false, "RCLiftExtend", "Command Complete, at requested position");
                    return true;
                }
                return false;
            }

        };
    }


    public void goLowBasket(){
        setPosition(LINEAR_LIFT_LOW_BASKET, 1);
    }

    public void goHighBasket(){
        setPosition(LINEAR_LIFT_HIGH_BASKET, 1);
    }

    public void goLowChamber(){
        setPosition(LINEAR_LIFT_LOW_CHAMBER, 1);
    }

    public void goHighChamber(){
        setPosition(LINEAR_LIFT_HIGH_CHAMBER, 1);
    }

    public void goSubmersible(){
        setPosition(LINEAR_LIFT_SUB_POS, 1);
    }

    public void goMin(){setPosition(LINEAR_LIFT_MIN_POS);}
    public void goMax() {
        setPosition(LINEARLIFT_MAXPOS);
    }

    public void calibrateLift() {
        hardware.logMessage(false, "Linear Lift", "Starting to calibrate Lift");
        state = LINEARLIFTNOTHOMED;
        if (home.isPressed()) {
            backOffHome();
        } else {
            findHome();
        }
    }

    public void findHome() {
        startTime = opMode.time;
        hardware.linearMotor.setMotorEnable();
        linearLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearLiftMotor.setPower(-1 * downLiftPower);
        hardware.logMessage(false, "Linear Lift", "Linear Lift State: Finding Home");
        state = LINEARLIFTFINDINGHOME;
    }

    public void backOffHome() {
        startTime = opMode.time;
        hardware.linearMotor.setMotorEnable();
        linearLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearLiftMotor.setPower(upLiftPower);
        hardware.logMessage(false, "Linear Lift", "Linear Lift State: Backing Off Home");
        state = LINEARLIFTBACKOFFHOME;
    }

    public void stop() {
        linearLiftMotor.setPower(0.0);
        linearLiftMotor.setVelocity(0.0);
        linearLiftMotor.setMotorDisable();
    }

    public int getState() {
        return state;
    }

    public int manualPositionIncrement(double power, int liftPreviousManualPosition){
        return liftPreviousManualPosition + (int) (power * LinearLift.LIFT_MANUAL_SPEED * hardware.getDeltaTime());
    }

    public void checkHorizontalLimit(int liftRotateCurrentPosition, int linearLiftCurrentPosition) {
        double rotationAngle = liftRotateCurrentPosition/2677.0 * 90.0;
//        opMode.telemetry.addData("Rotation Angle", rotationAngle);
//        rolling_max = (int)(42.0/48.0 * Math.cos(Math.toRadians(rotationAngle)) * 3825);
        rolling_max = (int)(40/Math.cos(Math.toRadians(rotationAngle)) * (3350.0/38.0));
        if(linearLiftCurrentPosition > rolling_max){
            setPosition(rolling_max);
        }
    }
}
