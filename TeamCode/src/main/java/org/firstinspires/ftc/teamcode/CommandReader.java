package org.firstinspires.ftc.teamcode;

public class CommandReader {

    String[] commandList;
    int instructionPointer = 0;

    CommandReader(String[] commands) {
        commandList = commands;
    }

    public void fetchNextInstructions(VirtualGamePad joystick1, VirtualGamePad joystick2) {
        /** Read next instructions from list; silently fail if done
         *
         */
        if (instructionPointer <= commandList.length-2) {
           joystick1.fromString(commandList[instructionPointer]);
           joystick2.fromString(commandList[instructionPointer+1]);
           instructionPointer += 2;
        }

    }
}
