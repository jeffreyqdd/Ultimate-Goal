package org.firstinspires.ftc.teamcode.team18036.util;

public class MathFunctions {
    public static double angleWrap(double angle) {
        while(angle < -Math.PI) {
            angle += 2d * Math.PI;
        }
        while(angle > Math.PI) {
            angle -= 2d * Math.PI;
        }

        return angle;
    }

    public static double round(double num, int numDecimals){
        return Math.round(num * Math.pow(10, numDecimals)) / Math.pow(10, numDecimals);
    }


}
