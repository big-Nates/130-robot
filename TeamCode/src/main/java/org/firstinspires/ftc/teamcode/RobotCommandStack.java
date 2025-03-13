package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

import java.util.ArrayList;
import java.util.List;

public class RobotCommandStack {

    private Hardware hardware = null;
    private String stackName = null;
    public RobotCommandStack(Hardware hardware, String stackName) {
//        this.opMode = opMode;
        this.stackName = stackName;
        this.hardware = hardware;
    }

    private final RobCommand robotCommandNull = new RobCommand();
    public int currentRobotCommandIndex = -1;
    public int nextRobotCommandIndex = 0;
    public List<RobCommand> robotCommands = new ArrayList<RobCommand>();

    public int currentRobotActionIndex = -1;
    public int nextRobotActionIndex = 0;
    public List<Action> robotActions = new ArrayList<Action>();




    public void processCommands(){
        if(currentRobotCommandIndex == -1){
            if(robotCommands.size() > nextRobotCommandIndex)
            {
                currentRobotCommandIndex = nextRobotCommandIndex;
                robotCommands.get(currentRobotCommandIndex).run();
            }
        }
        else if(currentRobotCommandIndex < robotCommands.size()){
            if(robotCommands.get(currentRobotCommandIndex).isComplete()){
                robotCommands.get(currentRobotCommandIndex).hasCompleted = true;
                nextRobotCommandIndex++;
                if(nextRobotCommandIndex < robotCommands.size()) {
                    currentRobotCommandIndex = nextRobotCommandIndex;
                    robotCommands.get(currentRobotCommandIndex).run();
                }
                else currentRobotCommandIndex = -1;
            }
        }
    }

    public void processActions(){
        if(currentRobotActionIndex == -1){
            if(robotActions.size() > nextRobotCommandIndex){
                currentRobotActionIndex = nextRobotActionIndex;
                runAction(robotActions.get(currentRobotActionIndex))
            }
        }
        else if(currentRobotActionIndex < robotCommands.size()){
            if(robotCommands.get(currentRobotActionIndex).isComplete()){
                robotActions.get(currentRobotActionIndex).run(new TelemetryPacket());
                nextRobotActionIndex++;
                if(nextRobotCommandIndex < robotCommands.size()) {
                    currentRobotActionIndex = nextRobotActionIndex;
                    runAction(robotActions.get(currentRobotActionIndex))
                }
                else currentRobotCommandIndex = -1;
            }
        }
    }

    public void runAction(Action action){
        if(action instanceOf RCRoadrunner1){
            Actions.runBlocking(action);
        }else{
            action.run();
        }
    }



    public void addCommand(RobCommand command){
        hardware.logMessage(false,"Robot130",stackName + "; Command Added: " + command.toString());
        robotCommands.add(command);
    }

    public void addAction(Action action){
        hardware.logMessage(false,"Robot130", stackName + "; Action Added: " + command.toString());
        robotActions.add(action)
    }

    public boolean isRoadRunnerActive(){
        if(getCurrentCommand() instanceof RCRoadrunner){
            return true;
        }else{
            return false;
        }
    }


    public int getNumCommands(){
        return robotCommands.size();
    }
    public int getCurrentCommandIndex(){
        return currentRobotCommandIndex;
    }
    public int getNextCommandIndex(){
        return nextRobotCommandIndex;
    }

    public RobCommand getCurrentCommand(){
        if(currentRobotCommandIndex == -1){
            return robotCommandNull;
        }
        return robotCommands.get(currentRobotCommandIndex);
    }

    public void cancelFutureCommands(){
        currentRobotCommandIndex = -1;
        nextRobotCommandIndex = robotCommands.size();
    }

}
