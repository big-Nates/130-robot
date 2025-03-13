package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "LiftLinearTest", group = "tests")
public class LinearLiftTest extends OpMode {

    private final Hardware hardware = new Hardware();

    private double targetPower = 1.0;
    private int positionIncrement = 100;
    private int targetPosition = 0;
    private int liftTargetPosition = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        hardware.init(hardwareMap, this);
//        hardware.pixelCabin.goToStowPosition();
//        hardware.pixelCabin.holdPixel();
    }

    @Override
    public void init_loop() {
        hardware.updateValues();

        super.init_loop();

        hardware.init_loop();
    }

    @Override
    public void loop() {
        double targetLPower = 0.0;
        double targetRPower = 0.0;
        double desiredLPower = 0.0;
        double desiredRPower = 0.0;
        float game2LeftY = hardware.gamepad2_current_left_stick_y;
        float game2RightY = hardware.gamepad2_current_right_stick_y;
        float game1LeftY = hardware.gamepad1_current_left_stick_y;
        float game1RightY = hardware.gamepad1_current_right_stick_y;
        double deltaExtension;
        double rightTriggerPosition = gamepad2.right_trigger;
        double leftTriggerPosition = gamepad2.left_trigger;
        double servoPower = 0;

        hardware.updateValues();

        // HOMING
        if (hardware.gamepad1_current_right_stick_button) {
            hardware.linearLift.calibrateLift();
        }
        if (hardware.gamepad1_current_left_stick_button) {
            hardware.linearLift.backOffHome();
        }

        if (hardware.gamepad1_current_x) {
            positionIncrement = 10;
        }
        if (hardware.gamepad1_current_y) {
            positionIncrement = 100;
        }
        if (hardware.gamepad1_current_b) {
            positionIncrement = 500;
        }

        if (hardware.gamepad1_current_a && !hardware.gamepad1_previous_a) {
            hardware.linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (hardware.gamepad1_current_dpad_up && !hardware.gamepad1_previous_dpad_up) {
            targetPosition += positionIncrement;
            hardware.linearLift.setPosition(targetPosition);
        }
        if (hardware.gamepad1_current_dpad_down && !hardware.gamepad1_previous_dpad_down) {
            targetPosition -= positionIncrement;
            hardware.linearLift.setPosition(targetPosition);
        }

        if (hardware.gamepad1_current_dpad_left && !hardware.gamepad1_previous_dpad_left) {
            hardware.linearLift.goMin();
        }
        if (hardware.gamepad1_current_dpad_right && !hardware.gamepad1_previous_dpad_right) {
            hardware.linearLift.goMax();
        }

        if (hardware.gamepad2_current_a && !hardware.gamepad2_previous_a) {
            hardware.linearLift.calibrateLift();
            targetPosition = 0;
        }

        if (hardware.gamepad2_current_left_stick_button && !hardware.gamepad2_previous_left_stick_button){
            if(targetPower >= 1.0){
                targetPower = 0.1;
            }else{
                targetPower += 0.1;
            }
        }

        if (hardware.gamepad1_current_right_stick_y > 0.03) {
            liftTargetPosition = hardware.linearLift.getCurrentPos() - 250;
            hardware.linearLift.setPosition(liftTargetPosition, targetPower);
        } else if (hardware.gamepad1_current_right_stick_y < -0.03) {
            liftTargetPosition = hardware.linearLift.getCurrentPos() + 250;
            hardware.linearLift.setPosition(liftTargetPosition, targetPower);
        }

        if (hardware.gamepad1_current_left_bumper && !hardware.gamepad1_previous_left_bumper) {
            hardware.linearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.linearMotor.setPower(1);
        }
        if (hardware.gamepad1_current_right_bumper && !hardware.gamepad1_previous_right_bumper) {
            hardware.linearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.linearMotor.setPower(-1);
        }

        hardware.loop();


        telemetry.addData("Increment", positionIncrement);
        telemetry.addData("Current Position", hardware.linearLift.getCurrentPos());
        telemetry.addData("Target Position", hardware.linearMotor.getTargetPosition());
        telemetry.addData("Target power", targetPower);

        telemetry.update();
    }

    @Override
    public void stop() {
        hardware.updateValues();

        hardware.logMessage(false, "MyFirstJava", "Stop Button Pressed");
        hardware.stop();
        super.stop();
    }

    @Override
    public void start() {
        hardware.updateValues();
        hardware.logMessage(false, "MyFirstJava", "Start Button Pressed");
        super.start();
        hardware.start();
    }
}
