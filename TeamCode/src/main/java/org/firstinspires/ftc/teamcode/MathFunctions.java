package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;

public class MathFunctions {



    public static double angleWrap(double angle) {

        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }

        return angle;

    }



}
