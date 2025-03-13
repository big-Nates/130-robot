package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

import androidx.annotation.NonNull;

@Config
@Autonomous(name = "Auto2324", group = "auto")
public class Auto2324 extends OpMode {
    private final Hardware hardware = new Hardware();
    private RobotConfiguration robotConfiguration = null;
    private boolean isRed = false;
    private boolean isLeftStartingPos = false;
    private boolean doParking = false;
    private int selectedSpikemark = -999;
    private int selectedTag = -999;
    private final int autoLiftPos = 670; //was originally 920
    private boolean SKIPCAMERA = true;
    private boolean commandsGrabbed = false;
    private final boolean firstRun = true;
    private final boolean secondRun = false;
    private final boolean testMode = false;
    private final boolean onlyPark = false;
    public double startTime = 0.0;

    private Pose2d startPose = null;


    @Override
    public void init() {
        System.gc();

        hardware.init(hardwareMap, this);

        robotConfiguration = new RobotConfiguration();
        robotConfiguration.readConfig();
        isRed = robotConfiguration.isRed;
        isLeftStartingPos = robotConfiguration.isLeftStartPos;
        doParking = robotConfiguration.doParking;

        telemetry.addLine("Configuration Fetched");
        telemetry.addData("Is Red?? ", isRed);
        telemetry.addData("Is Left Position? ", isLeftStartingPos);
        telemetry.addData("Park?", doParking);
        telemetry.update();

//        //Set Starting Position
//        startPose = isRed ? (isLeftStartingPos ? RED_LEFT_STARTPOS : RED_RIGHT_STARTPOS) : (isLeftStartingPos ? BLUE_LEFT_STARTPOS : BLUE_RIGHT_STARTPOS);
        if(isLeftStartingPos){
            startPose = new Pose2d(-31.25,-63, Math.toRadians(90));
            hardware.liftRotate.setPosition(1200);
        }
        else{
            startPose = new Pose2d(-7.25,-62.25, Math.toRadians(270));
            hardware.liftRotate.setPosition(1200);
        }

//        hardware.drive.setPoseEstimate(startPose);

        long zeroTime = System.currentTimeMillis() / 1000;
        // High Sample auto
        if(isLeftStartingPos) {

        }
        //High Specimen auto
        else{

        }

        commandsGrabbed = true;
        telemetry.addData("Trajectory Creation", "Is complete");
        telemetry.update();

    }

    @Override
    public void init_loop() {
        hardware.updateValues();

        super.init_loop();

        hardware.init_loop();
    }



    @Override
    public void loop() {
        hardware.updateValues();
        hardware.loop();
        if(!commandsGrabbed){

        }


        if (commandsGrabbed) {
            hardware.robo130.processCommands();
        }

        telemetry.addData("x", hardware.odom.getPosX() / 25.4);
        telemetry.addData("y", hardware.odom.getPosY() / 25.4);
        telemetry.addData("Selected Tag: ", selectedTag);
        telemetry.addData("Robot Command Stick: ", Integer.toString(hardware.robo130.robotCommandStack.getNumCommands())
                + " " + Integer.toString(hardware.robo130.robotCommandStack.getCurrentCommandIndex())
                + " " + Integer.toString(hardware.robo130.robotCommandStack.getNextCommandIndex()));
        telemetry.addData("Roadrunner Command Stick: ", Integer.toString(hardware.robo130.roadrunnerCommandStack.getNumCommands())
                + " " + Integer.toString(hardware.robo130.roadrunnerCommandStack.getCurrentCommandIndex())
                + " " + Integer.toString(hardware.robo130.roadrunnerCommandStack.getNextCommandIndex()));
        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    public void start() {
//        hardware.webcamPipeline.saveProcessedImages();
//        hardware.webcam.stopStreaming();
        hardware.updateValues();
        hardware.logMessage(false, "Auto2324", "Start Button Pressed");
        super.start();
        hardware.start();

    }

    public void stop() {
        hardware.updateValues();
        hardware.logMessage(false, "Auto2324", "Stop Button Pressed");
        hardware.stop();
        super.stop();
    }

    public double calculateWaitForLift(){
        if(hardware.liftRotate.getCurrentPos() < -1){

        }
        return 0;
    }
}
