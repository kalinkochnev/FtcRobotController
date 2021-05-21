package org.firstinspires.ftc.teamcode.helpers;


/**
 * Contains a sequence of moves and executes them.
 **/
public class MoveSequence {
    private Robot robot;
    private double[][] moves;

    /**
     * Array of length-2 arrays where the first value is the distance and the second is the angle hold.
     **/
    public MoveSequence(Robot robot, double[][] moves) {
        this.robot = robot;
        this.moves = moves;
    }

    public MoveSequence(double[][] inputs) {
    }

    public void execute() {
        for (int i = 0; i < moves.length; i++) {
            if (moves[i][0] == 0) {
                robot.rotate(0.3, moves[i][1]);
            }
            else {
                robot.moveCardinal(0.3, inchToMeter(moves[i][0]), moves[i][1]);
            }
        }
    }

    public static double inchToMeter(double i){
        return i*0.0254;
    }
}