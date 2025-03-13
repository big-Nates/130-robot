package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "LiftRotateTest", group = "tests")
public class LiftRotateTest extends OpMode {
    private final Hardware hardware = new Hardware();

    private int positionIncrement = 10;
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
            hardware.liftRotate.calibrateLift();
        }
        if (hardware.gamepad1_current_left_stick_button) {
            hardware.liftRotate.backOffHome();
        }

        if (hardware.gamepad1_current_x) {
            positionIncrement = 1;
        }
        if (hardware.gamepad1_current_y) {
            positionIncrement = 10;
        }
        if (hardware.gamepad1_current_b) {
            positionIncrement = 50;
        }

        if (hardware.gamepad1_current_a && !hardware.gamepad1_previous_a) {
            hardware.rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (hardware.gamepad1_current_dpad_up && !hardware.gamepad1_previous_dpad_up) {
            targetPosition += positionIncrement;
            hardware.liftRotate.setPosition(targetPosition);
        }
        if (hardware.gamepad1_current_dpad_down && !hardware.gamepad1_previous_dpad_down) {
            targetPosition -= positionIncrement;
            hardware.liftRotate.setPosition(targetPosition);
        }

        if (hardware.gamepad1_current_dpad_left && !hardware.gamepad1_previous_dpad_left) {
            hardware.liftRotate.goMin();
        }
        if (hardware.gamepad1_current_dpad_right && !hardware.gamepad1_previous_dpad_right) {
            hardware.liftRotate.goMax();
        }

        if (hardware.gamepad2_current_a & !hardware.gamepad2_previous_a) {
            hardware.liftRotate.calibrateLift();
            targetPosition = 0;
        }

        if (hardware.gamepad1_current_right_stick_y > 0.03) {
            liftTargetPosition = hardware.liftRotate.getCurrentPos() - 250;
            hardware.liftRotate.setPosition(liftTargetPosition);
        } else if (hardware.gamepad1_current_right_stick_y < -0.03) {
            liftTargetPosition = hardware.liftRotate.getCurrentPos() + 250;
            hardware.liftRotate.setPosition(liftTargetPosition);
        }

        if (hardware.gamepad1_current_left_bumper && !hardware.gamepad1_previous_left_bumper) {
            hardware.rotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.rotateMotor.setPower(0.5);
        }
        if (hardware.gamepad1_current_right_bumper && !hardware.gamepad1_previous_right_bumper) {
            hardware.rotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.rotateMotor.setPower(-0.5);
        }

        hardware.loop();


        telemetry.addData("Home Pressed?", hardware.liftRotateHome.isPressed());
        telemetry.addData("Current Position", hardware.liftRotate.getCurrentPos());
        telemetry.addData("Target Position", hardware.rotateMotor.getTargetPosition());

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
        hardware.specimenClaw.closeClaw();
        hardware.logMessage(false, "MyFirstJava", "Start Button Pressed");
        super.start();
        hardware.start();
    }
}