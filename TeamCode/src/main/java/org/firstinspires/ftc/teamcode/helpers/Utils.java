package org.firstinspires.ftc.teamcode.helpers;

public class Utils {
    public static double getMin(double[] arr) {
        double min = arr[0];
        for (int i = 1; i < arr.length; i++) {
            if (arr[i] < min) {
                min = arr[i];
            }
        }
        return min;
    }

    public static double getMax(double[] arr) {
        double max = arr[0];
        for (int i = 1; i < arr.length; i++) {
            if (arr[i] > max) {
                max = arr[i];
            }
        }
        return max;
    }

    public static double[] absoluteValues(int[] arr) {
        return absoluteValues(toDoubleArr(arr));
    }
    public static double[] absoluteValues(double[] arr) {
        double[] newArr = arr.clone();
        for (int i = 0; i < arr.length; i++) {
            newArr[i] = Math.abs(arr[i]);
        }
        return newArr;
    }

    public static double getSum(double[] arr) {
        double sum = 0;
        for (int i = 0; i < arr.length; i++) {
            sum += arr[i];
        }
        return sum;
    }

    public static double[] toDoubleArr(int[] arr) {
        double[] newArr = new double[arr.length];
        for (int i = 0; i < arr.length; i++) {
            newArr[i] = (double)arr[i];
        }
        return newArr;
    }

    public static int[] toIntArr(double[] arr) {
        int[] newArr = new int[arr.length];
        for (int i = 0; i < arr.length; i++) {
            newArr[i] = (int)arr[i];
        }
        return newArr;
    }

}
