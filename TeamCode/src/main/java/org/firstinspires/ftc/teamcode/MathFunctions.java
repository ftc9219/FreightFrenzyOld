package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;

import java.util.ArrayList;

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

    public static ArrayList<Point> lineCircleIntersection(Point one, Point two) {

        Hardware hardware = new Hardware();

        double radius = 1;
        double gx = hardware.GlobalPos[0];
        double gy = hardware.GlobalPos[1];

        double m = (two.y - one.y) / (two.x - one.x);
        double y1 = one.y - gy;
        double x1 = one.x - gx;
        double b = y1 - (m * x1);
        double xIntercept1 = ((-b * m) + Math.sqrt(Math.pow(radius,2) + (Math.pow(m,2) * Math.pow(radius,2)) * Math.pow(b, 2))) / (Math.pow(m, 2) + 1);
        double yIntercept1 = (m * xIntercept1)) + b;

    }


}
