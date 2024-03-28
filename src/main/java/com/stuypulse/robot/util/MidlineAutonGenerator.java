package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj2.command.Command;

public class MidlineAutonGenerator extends Command {
    private final char[] noteOrder;
    private final Command[] decisionTree;
    private int index = 0;

    public MidlineAutonGenerator(char... noteOrder) {
        this.noteOrder = noteOrder;
        decisionTree = new Command[(1 << noteOrder.length) + 1];
        int i = 0;
        // Implement creating the decision tree
        // while (i < noteOrder.length) {
        //     decisionTree[2 * i + 1] = new 
        // }
    }

    public MidlineAutonGenerator(String noteOrder) {
        this(noteOrder.toCharArray());
    }

    public Command getNextCommand(boolean hasNote) {
        index = 2 * index + (hasNote ? 1 : 2);
        return decisionTree[index];
    }
}
