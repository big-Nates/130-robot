/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="OpMode 24-25", group="Linear OpMode")
public class OpMode2425 extends OpMode {
    private final Hardware hardware = new Hardware();

    private static final double STRAFE_POWER = 0.50;
    private double prevLPower = 0.0;
    private double prevRPower = 0.0;

    private final boolean isAccelDriveMode = false;

    private RobotConfiguration robotConfiguration = null;

    private boolean isRed = false;
    private boolean isLeftStartingPos = false;

    private final boolean isPreviousManualDrive = false;
    private final boolean isCurrentManualDrive = false;
    private double currentGasPedalPower = 1.0;
    private boolean reverseControls = false;

    private int liftTargetPosition = 0;
    private int rotateLiftTargetPosition = 0;
    private boolean liftManualMode = false;

    private boolean liftRotateManualMode = false;


    private boolean rotateLiftManualMode = false;
    private int liftPreviousManualPosition = LinearLift.LINEAR_LIFT_MIN_POS;

    private int liftPreviousRotationPosition = LiftRotate.LIFT_MIN_POS;

    private Pose2d startPose = null;



//    private int liftTargetPosition = 0;
//    private boolean liftManualMode = false;
//    private int liftPreviousManualPosition = LinearLift.LIFT_MINPOS;
//    private boolean manualIntake = false;

    @Override
    public void init() {
        System.gc();

        hardware.init(hardwareMap, this);

        robotConfiguration = new RobotConfiguration();
        robotConfiguration.readConfig();
        isRed = robotConfiguration.isRed;
        isLeftStartingPos = robotConfiguration.isLeftStartPos;
        double startingAngle = 90;

        startPose = new Pose2d(0,0,Math.toRadians(0));
        telemetry.addLine("Configuration Fetched");
//        hardware.drive.setPoseEstimate(startPose);
//        hardware.odom.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90));
//        telemetry.addData("Is Red?? ", isRed);
//        telemetry.addData("Is Left Position? ", isLeftStartingPos);
        telemetry.update();

//        startPose = new Pose2d(0, 0);
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
        double targetLiftPower = 0.0;
        double desiredLiftPower = 0.0;
        double targetArmPower = 0.0;
        double desiredArmPower = 0.0;
        double targetSpinPower = 0.0;
        float game2LeftY = hardware.gamepad2_current_left_stick_y;
        float game2RightY = hardware.gamepad2_current_right_stick_y;
        float game1LeftY = hardware.gamepad1_current_left_stick_y;
        float game1LeftX = hardware.gamepad1_current_left_stick_x;
        float game1RightY = hardware.gamepad1_current_right_stick_y;
        float game1RightX = hardware.gamepad1_current_right_stick_x;
        double deltaExtension;
        double servoPower = 0;
        double currentGasPedal = 1.0;

        hardware.updateValues();

        //GAMEPAD_1
        //Gas Pedal
        if (hardware.gamepad1_current_left_trigger < 0.05 && hardware.gamepad1_current_right_trigger < 0.05) {
            currentGasPedalPower = 1.0;
        } else if (hardware.gamepad1_current_left_trigger > 0.5) {
            currentGasPedalPower = (Math.max(1.0 - hardware.gamepad1_current_left_trigger, 0.15));
        } else if (hardware.gamepad1_current_right_trigger > 0.5) {
            currentGasPedalPower = (Math.max(1.0 - hardware.gamepad1_current_right_trigger, 0.6));
        }




        //Roadrunner Drive Controls
        // Read pose
//        Pose2d poseEstimate = hardware.drive.getPoseEstimate();

//        if(hardware.robo130.isRoadrunnerActive()){
        if(hardware.enableTeleopDrive){
            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
//            Vector2d input = new Vector2d(
//                    -hardware.gamepad1_current_left_stick_y,
//                    -hardware.gamepad1_current_left_stick_x
//            ).rotated(-poseEstimate.getHeading());
//
//            // Pass in the rotated input + right stick value for rotation
//            // Rotation is not part of the rotated input thus must be passed in separately
//            hardware.drive.setWeightedDrivePower(
//                    new Pose2d(
//                            input.getX() * currentGasPedalPower,
//                            input.getY() * currentGasPedalPower,
//                            -hardware.gamepad1_current_right_stick_x * currentGasPedalPower
//                    )
//            );
        }

//        }


        //Old Roadrunner Drive Controls
//                hardware.drive.setWeightedDrivePower(new Pose2d(
//                hardware.gamepad1_current_left_stick_x * currentGasPedalPower,
//                -1 * hardware.gamepad1_current_left_stick_y * currentGasPedalPower,
//                hardware.gamepad1_current_right_stick_x * currentGasPedalPower
//        ));




        //COMMANDS
//        if (hardware.gamepad1_current_x && !hardware.gamepad1_previous_x) {
//            hardware.robo130.cancelFutureCommands(); //XXX LOOK AT THIS LATER, THIS WILL PROBABLY BREAK THE ROBOT IN THE FUTURE LOL
//        }



        //Sample Intake Mech
        if (hardware.gamepad2_current_right_trigger > 0.03 && hardware.gamepad2_previous_right_trigger == 0){
            hardware.robo130.addCommand(new RCSampleIntake(hardware, RCSampleIntake.CMD_TOGGLE_POWER, true, true));
        }else if(hardware.gamepad2_current_left_trigger > 0.03 && hardware.gamepad2_previous_left_trigger == 0){
            hardware.robo130.addCommand(new RCSampleIntake(hardware, RCSampleIntake.CMD_TOGGLE_POWER, false, true));
        }


        if(hardware.gamepad2_current_y && !hardware.gamepad2_previous_y){   //Go High Chamber
            hardware.robo130.addCommand(new RCLiftRotate(hardware, LiftRotate.LIFT_VERT_POS, 1.0));
            hardware.robo130.addCommand(new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_ABOVE_HIGH_CHAMBER, 1.0));
        }
        if(hardware.gamepad2_current_dpad_down && !hardware.gamepad2_previous_dpad_down){   //Score High Chamber
            hardware.robo130.addCommand(new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_HIGH_CHAMBER, 1.0));
            hardware.robo130.addCommand(new RCSpecimenClaw(hardware, RCSpecimenClaw.CMD_OPEN, false));
//            hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(new Pose2d())
//                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
//                    .forward(5)
//                    .resetAccelConstraint()
//                    .build()
//            ));
//            hardware.robo130.addCommand(new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_WALL_POS, 1.0));
        }
        if(hardware.gamepad2_current_dpad_left && !hardware.gamepad2_previous_dpad_left){   //Collect Specimen (pt. 1)
            hardware.robo130.addCommand(new RCSpecimenClaw(hardware, RCSpecimenClaw.CMD_OPEN, true));
            hardware.robo130.addCommand(new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_STOW_POS, 1.0));
            hardware.robo130.addCommand(new RCLiftRotate(hardware, LiftRotate.LIFT_VERT_POS, 1.0));
//            hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(new Pose2d())
//                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
//                    .strafeLeft(24)
//                    .resetAccelConstraint()
//                    .build()
//            ));
        }
        if(hardware.gamepad2_current_dpad_right && !hardware.gamepad2_previous_dpad_right ){    //Grip Specimen (pt. 2)
            hardware.robo130.addCommand(new RCSpecimenClaw(hardware, RCSpecimenClaw.CMD_CLOSE, false));
            hardware.robo130.addCommand(new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_ABOVE_WALL, 1.0));
//            hardware.robo130.addCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(new Pose2d())
//                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * .75))
//                    .strafeRight(24)
//                    .resetAccelConstraint()
//                    .build()
//            ));
        }

        //Linear Slide Mechanism


        //Manual Linear Slide Controls
        {
            if (!liftManualMode && Math.abs(hardware.gamepad2_current_left_stick_y) > 0.03) {
                liftManualMode = true;
                liftPreviousManualPosition = hardware.linearLift.getCurrentPos();
                liftTargetPosition = hardware.linearLift.manualPositionIncrement(hardware.gamepad2_current_left_stick_y * -1, liftPreviousManualPosition);
                liftPreviousManualPosition = liftTargetPosition;
                hardware.linearLift.setPosition(liftTargetPosition);
            } else if (liftManualMode && Math.abs(hardware.gamepad2_current_left_stick_y) > 0.03) {
                liftTargetPosition = hardware.linearLift.manualPositionIncrement(hardware.gamepad2_current_left_stick_y * -1, liftPreviousManualPosition);
                liftPreviousManualPosition = liftTargetPosition;
                hardware.linearLift.setPosition(liftTargetPosition);
            } else if (liftManualMode && Math.abs(hardware.gamepad2_current_left_stick_y) < 0.03) {
                liftPreviousManualPosition = hardware.linearLift.getCurrentPos();
                liftTargetPosition = liftPreviousManualPosition;
                hardware.linearLift.setPosition(liftTargetPosition);
                liftManualMode = false;
            }
            //Manual Linear Slide Rotation Controls
            if (!liftRotateManualMode && Math.abs(hardware.gamepad2_current_right_stick_y) > 0.03) {
                liftRotateManualMode = true;
                liftPreviousRotationPosition = hardware.liftRotate.getCurrentPos();
                rotateLiftTargetPosition = hardware.liftRotate.manualPositionIncrement(hardware.gamepad2_current_right_stick_y * -1, liftPreviousRotationPosition);
                liftPreviousRotationPosition = rotateLiftTargetPosition;
                hardware.liftRotate.setPosition(rotateLiftTargetPosition);
            } else if (liftRotateManualMode && Math.abs(hardware.gamepad2_current_right_stick_y) > 0.03) {
                rotateLiftTargetPosition = hardware.liftRotate.manualPositionIncrement(hardware.gamepad2_current_right_stick_y * -1, liftPreviousRotationPosition);
                liftPreviousRotationPosition = rotateLiftTargetPosition;
                hardware.liftRotate.setPosition(rotateLiftTargetPosition);
            } else if (liftRotateManualMode && Math.abs(hardware.gamepad2_current_right_stick_y) < 0.03) {
                liftPreviousRotationPosition = hardware.liftRotate.getCurrentPos();
                rotateLiftTargetPosition = liftPreviousRotationPosition;
                hardware.liftRotate.setPosition(rotateLiftTargetPosition);
                liftRotateManualMode = false;
            }
        }



        //Automatic Controls
        if(hardware.gamepad2_current_a && !hardware.gamepad2_previous_a){ //Goes to Submersible Zone
            rotateLiftManualMode = false;
            hardware.robo130.addCommand(new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_STOW_POS, 1.0));
            hardware.robo130.addCommand(new RCLiftRotate(hardware, LiftRotate.LIFT_SUB_POS, 1.0));
        }else if(hardware.gamepad2_current_b && !hardware.gamepad2_previous_b){ //Goes to High Basket
            rotateLiftManualMode = false;
            hardware.robo130.addCommand(new RCLiftRotate(hardware, LiftRotate.LIFT_HIGH_BASKET, 1.0));
            hardware.robo130.addCommand(new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_HIGH_BASKET, 1.0));
        }else if(hardware.gamepad2_current_x && !hardware.gamepad2_previous_x){ //Goes to Low Basket
            rotateLiftManualMode = false;
            hardware.robo130.addCommand(new RCLiftRotate(hardware, LiftRotate.LIFT_LOW_BASKET     , 1.0));
            hardware.robo130.addCommand(new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_LOW_BASKET, 1.0));
        }

        if(hardware.liftRotate.getCurrentPos() <= LiftRotate.LIFT_SUB_POS+500){
            hardware.linearLift.useHorizontalMax();
        }else{
            hardware.linearLift.useMax();
        }
//        hardware.linearLift.checkHorizontalLimit(hardware.liftRotate.getCurrentPos());

        if(hardware.gamepad1_current_y && !hardware.gamepad1_previous_y){
            hardware.robo130.robotCommandStack.cancelFutureCommands();
            hardware.robo130.roadrunnerCommandStack.cancelFutureCommands();
            hardware.linearLift.calibrateLift();
        }

        if(hardware.gamepad1_current_a && !hardware.gamepad1_previous_a){
//            hardware.odom.resetPosAndIMU();
//            startPose = new Pose2d(Units.mmToInches(hardware.odom.getPosX()), Units.mmToInches(hardware.odom.getPosY()), Math.toDegrees(hardware.odom.getHeading()));
        }

//        if(hardware.gamepad1_current_dpad_left && !hardware.gamepad1_previous_dpad_left){
//            hardware.enableTeleopDrive = false;
//            hardware.drive.setPoseEstimate(new Pose2d(-22.75, 11.25, Math.toRadians(0)));
//            //new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_HIGH_BASKET, 1.0)
//            RobCommand atBasket = new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_HIGH_BASKET, 1.0);
//            RobCommand compressedLift = new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_MIN_POS, 1.0);
//            hardware.robo130.addRoadrunnerCommand(new RCLiftRotate(hardware, LiftRotate.LIFT_SUB_POS, 1.0));
//            hardware.robo130.addRoadrunnerCommand(compressedLift);
//            hardware.robo130.addRoadrunnerCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(new Pose2d(-22.75, -11.25, Math.toRadians(0)))
//                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5))
//                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                    .splineToLinearHeading(new Pose2d(-55.5, -66.75, Math.toRadians(225)), Math.toRadians(180))
//                    .addTemporalMarker(0.1, () -> {
//                        hardware.robo130.addToFourthStack(new RCLiftRotate(hardware, LiftRotate.LIFT_HIGH_BASKET, 1.0));
//                        hardware.robo130.addToThirdStack(new RCWait(hardware, 0.75));
//                        hardware.robo130.addToThirdStack(new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_HIGH_BASKET, 1.0));
//                        hardware.robo130.addToThirdStack(new RCRoadrunnerSync(hardware, atBasket));
//                        hardware.robo130.addToThirdStack(new RCSampleIntake(hardware, RCSampleIntake.CMD_ON, true));
//                    })
//                    .resetAccelConstraint()
//                    .resetVelConstraint()
//                    .build()
//            ));
//            hardware.robo130.addRoadrunnerCommand(new RCEnableTeleop(hardware));
//        }
//
//        if(hardware.gamepad1_current_dpad_right && !hardware.gamepad1_previous_dpad_right){
//            hardware.enableTeleopDrive = false;
//            hardware.drive.setPoseEstimate(new Pose2d(-22.75, 0, Math.toRadians(0)));
//            RobCommand atBasket = new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_HIGH_BASKET, 1.0);
//            RobCommand compressedLift = new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_MIN_POS, 1.0);
//            hardware.robo130.addRoadrunnerCommand(new RCLiftRotate(hardware, LiftRotate.LIFT_SUB_POS, 1.0));
//            hardware.robo130.addRoadrunnerCommand(compressedLift);
//            hardware.robo130.addRoadrunnerCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(new Pose2d(-22.75, 11.25, Math.toRadians(0)))
//                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5))
//                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                    .splineToLinearHeading(new Pose2d(-55.5, -66.75, Math.toRadians(225)), Math.toRadians(180))
//                    .addTemporalMarker(0.1, () -> {
//                        hardware.robo130.addToFourthStack(new RCLiftRotate(hardware, LiftRotate.LIFT_HIGH_BASKET, 1.0));
//                        hardware.robo130.addToThirdStack(new RCWait(hardware, 0.75));
//                        hardware.robo130.addToThirdStack(new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_HIGH_BASKET, 1.0));
//                        hardware.robo130.addToThirdStack(new RCRoadrunnerSync(hardware, atBasket));
//                        hardware.robo130.addToThirdStack(new RCSampleIntake(hardware, RCSampleIntake.CMD_ON, true));
//                    })
//                    .resetAccelConstraint()
//                    .resetVelConstraint()
//                    .build()
//            ));
//            hardware.robo130.addRoadrunnerCommand(new RCEnableTeleop(hardware));
//        }
//
//        if(hardware.gamepad1_current_dpad_up && !hardware.gamepad1_previous_dpad_up){
//            hardware.enableTeleopDrive = false;
//            hardware.drive.setPoseEstimate(new Pose2d(-22.75, -11.250, Math.toRadians(0)));
//            RobCommand atBasket = new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_HIGH_BASKET, 1.0);
//            RobCommand compressedLift = new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_MIN_POS, 1.0);
//            hardware.robo130.addRoadrunnerCommand(new RCLiftRotate(hardware, LiftRotate.LIFT_SUB_POS, 1.0));
//            hardware.robo130.addRoadrunnerCommand(compressedLift);
//            hardware.robo130.addRoadrunnerCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(new Pose2d(-22.75, 0, Math.toRadians(0)))
//                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5))
//                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                    .splineToLinearHeading(new Pose2d(-55.5, -66.75, Math.toRadians(225)), Math.toRadians(180))
//                    .addTemporalMarker(0.1, () -> {
//                        hardware.robo130.addToFourthStack(new RCLiftRotate(hardware, LiftRotate.LIFT_HIGH_BASKET, 1.0));
//                        hardware.robo130.addToThirdStack(new RCWait(hardware, 0.75));
//                        hardware.robo130.addToThirdStack(new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_HIGH_BASKET, 1.0));
//                        hardware.robo130.addToThirdStack(new RCRoadrunnerSync(hardware, atBasket));
//                        hardware.robo130.addToThirdStack(new RCSampleIntake(hardware, RCSampleIntake.CMD_ON, true));
//                    })
//                    .resetAccelConstraint()
//                    .resetVelConstraint()
//                    .build()
//            ));
//            hardware.robo130.addRoadrunnerCommand(new RCEnableTeleop(hardware));
//        }
//
//        if(hardware.gamepad1_current_dpad_down && !hardware.gamepad1_previous_dpad_down){
//            hardware.enableTeleopDrive = false;
//            hardware.drive.setPoseEstimate(RCRoadrunner.getPreviousEndPoint());
//            RobCommand atBasket = new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_HIGH_BASKET, 1.0);
//            RobCommand compressedLift = new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_MIN_POS, 1.0);
//            hardware.robo130.addRoadrunnerCommand(new RCLiftRotate(hardware, LiftRotate.LIFT_SUB_POS, 1.0));
//            hardware.robo130.addRoadrunnerCommand(compressedLift);
//            hardware.robo130.addRoadrunnerCommand(new RCRoadrunner(hardware, hardware.drive.trajectorySequenceBuilder(RCRoadrunner.getPreviousEndPoint())
//                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5))
//                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                    .splineToLinearHeading(new Pose2d(-55.5, -66.75, Math.toRadians(225)), Math.toRadians(180))
//                    .addTemporalMarker(0.1, () -> {
//                        hardware.robo130.addToFourthStack(new RCLiftRotate(hardware, LiftRotate.LIFT_HIGH_BASKET, 1.0));
//                        hardware.robo130.addToThirdStack(new RCWait(hardware, 0.75));
//                        hardware.robo130.addToThirdStack(new RCLiftExtend(hardware, LinearLift.LINEAR_LIFT_HIGH_BASKET, 1.0));
//                        hardware.robo130.addToThirdStack(new RCRoadrunnerSync(hardware, atBasket));
//                        hardware.robo130.addToThirdStack(new RCSampleIntake(hardware, RCSampleIntake.CMD_ON, true));
//                    })
//                    .resetAccelConstraint()
//                    .resetVelConstraint()
//                    .build()
//            ));
//            hardware.robo130.addRoadrunnerCommand(new RCEnableTeleop(hardware));
//        }




        hardware.robo130.processCommands();

        hardware.loop();

        prevLPower = targetLPower;
        prevRPower = targetRPower;

//        telemetry.addData("Front Distance", hardware.frontDistance.getDistance(DistanceUnit.INCH));
//        telemetry.addData("Rear Distance", hardware.rearDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Delta Time", hardware.getDeltaTime());
        telemetry.addData("Robot Command Stack: ", Integer.toString(hardware.robo130.robotCommandStack.getNumCommands())
                + " " + Integer.toString(hardware.robo130.robotCommandStack.getCurrentCommandIndex())
                + " " + Integer.toString(hardware.robo130.robotCommandStack.getNextCommandIndex()));
        telemetry.addData("Roadrunner Command Stack: ", Integer.toString(hardware.robo130.roadrunnerCommandStack.getNumCommands())
                + " " + Integer.toString(hardware.robo130.roadrunnerCommandStack.getCurrentCommandIndex())
                + " " + Integer.toString(hardware.robo130.roadrunnerCommandStack.getNextCommandIndex()));
        telemetry.addData("Status", "Running");
        telemetry.addData("Lift position: ", hardware.linearLift.getCurrentPos());
        telemetry.addData("Lift rotate position: ", hardware.liftRotate.getCurrentPos());
//        telemetry.addData("X position: ", hardware.odom.getPosX() / 25.4);
//        telemetry.addData("Y position: ", hardware.odom.getPosY() / 25.4);
//        telemetry.addData("Heading: ", hardware.odom.getHeading());
//        telemetry.addData("Intake state: ", hardware.sampleIntake.getStateString());
//        telemetry.addData("Specimen state: ", hardware.specimenClaw.getStateString());
        telemetry.update();
    }

    @Override
    public void stop() {
        hardware.updateValues();

        hardware.logMessage(false, "OpMode2223", "Stop Button Pressed");
        hardware.stop();
        super.stop();
    }

    @Override
    public void start() {
        hardware.updateValues();
        hardware.logMessage(false, "OpMode2223", "Start Button Pressed");
        super.start();
        hardware.start();
    }
    }
