package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Specimen Claw Test", group = "tests")
//@Disabled
public class SpecimenClawTest extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final Hardware hardware = new Hardware();
    private double specimenClawPosition = 0.5;

    private Servo specimenClawServo = null;

    private double positionIncrement = .1;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        hardware.init(hardwareMap, this);

        specimenClawServo = hardware.specimenServo;

        telemetry.addData("Specimen Claw Servo Position", specimenClawPosition);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        hardware.updateValues();

        super.init_loop();

        hardware.init_loop();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        hardware.updateValues();

        hardware.logMessage(false, "MyFirstJava", "Start Button Pressed");
        super.start();
        hardware.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        hardware.updateValues();

        if (hardware.gamepad1_current_x) {
            positionIncrement = 0.001;
        }
        if (hardware.gamepad1_current_y) {
            positionIncrement = .01;
        }
        if (hardware.gamepad1_current_b) {
            positionIncrement = .1;
        }

        if (hardware.gamepad1_current_dpad_up & !hardware.gamepad1_previous_dpad_up) {
            specimenClawPosition = Math.min(Math.max((specimenClawPosition + positionIncrement), 0.0), 1.0);
        } else if (hardware.gamepad1_current_dpad_down & !hardware.gamepad1_previous_dpad_down) {
            specimenClawPosition = Math.min(Math.max((specimenClawPosition - positionIncrement), 0.0), 1.0);
        }
        hardware.specimenClaw.setClawPosition(specimenClawPosition);

        hardware.loop();

        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Specimen Claw Servo Position", specimenClawPosition);
        telemetry.addData("Specimen Claw Servo Hardware Position", specimenClawServo.getPosition());
        telemetry.addData("Specimen Claw Servo increment", positionIncrement);
        telemetry.update();
    }


    @Override
    public void stop() {
        hardware.updateValues();

        hardware.logMessage(false, "MyFirstJava", "Stop Button Pressed");
        hardware.stop();
        super.stop();
    }
}