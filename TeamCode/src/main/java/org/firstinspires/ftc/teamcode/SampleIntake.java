package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SampleIntake {
    private OpMode opMode;
    private Hardware hardware;
    private DcMotorEx intakeMotor;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime timeout = new ElapsedTime();

    private final double INTAKEPOWER = 0.50;
    private final double OUTTAKEPOWER = -0.4;
    private double startTime = 0;

    private static final double MAX_TIMEOUT = 5.0;

    public static final int STATIONARY = 0;
    public static final int INTAKING = 1;
    public static final int OUTTAKING = 2;
    private int state = STATIONARY;
    private boolean isRotating = false;

    public SampleIntake(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init(){
        opMode.telemetry.addData("Intake Status", "Initializing");
        intakeMotor = hardware.intakeMotor;
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        runtime.reset();
        timeout.reset();

        opMode.telemetry.addData("Intake Status", "Initialized");
        opMode.telemetry.update();
    }



    public void toggleDirection(String motive){
        if(motive.equals("Intake")){
            switch(state) {
                case INTAKING:
                    noRotation();
                    break;

                case OUTTAKING:
                    intakeRotation();
                    break;

                case STATIONARY:
                    intakeRotation();
                    break;
            }
        }else if(motive.equals("Outtake")){
            switch(state) {
                case INTAKING:
                    outtakeRotation();
                    break;

                case OUTTAKING:
                    noRotation();
                    break;

                case STATIONARY:
                    outtakeRotation();
                    break;
            }
        }
    }

    public String getStateString(){
        switch(state){
            case INTAKING:
                return "Intaking";
            case OUTTAKING:
                return "Outtaking";
            case STATIONARY:
                return "Stationary";
        }
        return null;
    }

    public int getState(){
        return state;
    }

    public void setPower(double power){
        intakeMotor.setPower(power);
        if(power > 0){
            state = INTAKING;
        }else if(power < 0){
            state = OUTTAKING;
        }else{
            noRotation();
        }
    }

    public void intakeRotation(){
        intakeMotor.setPower(INTAKEPOWER);
        state = INTAKING;
    }

    public void outtakeRotation(){
        intakeMotor.setPower(OUTTAKEPOWER);
        state = OUTTAKING;
    }

    public void noRotation(){
        intakeMotor.setPower(0);
        state = STATIONARY;
    }

}
