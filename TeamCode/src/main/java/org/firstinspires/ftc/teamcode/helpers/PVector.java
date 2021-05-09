package org.firstinspires.ftc.teamcode.helpers;

public class PVector {
    double x;
    double y;

    PVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    static PVector fromAngle(double magnitude, double angle) {
        return new PVector(magnitude * Math.cos(angle), magnitude * Math.sin(angle));
    }

    public static PVector add(PVector[] vectorArr) {
        double sumX = 0;
        double sumY = 0;
        for (int i = 0; i < vectorArr.length; i++) {
            sumX += vectorArr[i].x;
            sumY += vectorArr[i].y;
        }
        return new PVector(sumX, sumY);
    }
    public static PVector add(PVector p1, PVector p2) {
        return PVector.add(new PVector[]{p1, p2});
    }

    public PVector scalarDivide(double amount) {
        return new PVector(this.x/amount, this.y/amount);
    }

    public double getMagnitude() {
        return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
    }

    public static double dotProd(PVector p1, PVector p2) {
        return p1.x * p2.x + p1.y * p2.y;
    }
    public String toString() {
        return "x: " + this.x + " --- " + "y: " + this.y + " --- " + this.getMagnitude();
    }
    public boolean equals(PVector other) {
        return (this.x == other.x) && (this.y == other.y);
    }
}
