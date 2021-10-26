package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;

public class MathFunctions {

    public Point lineCircleIntercept(Point one, Point two, double gx, double gy, double radius) {
        double m = (two.y - one.y) / (two.x - one.x);
        double y1 = one.y - gy;
        double x1 = one.x - gx;
        double b = y1 - (m * x1);
        double quadratic = Math.pow(radius, 2) + (Math.pow(m, 2) * Math.pow(radius, 2)) - Math.pow(b, 2);

        if (quadratic >= 0) {
            double xIntercept1 = ((-b * m) + Math.sqrt(quadratic)) / (Math.pow(m, 2) + 1);
            double yIntercept1 = (m * xIntercept1) + b;
            xIntercept1 += gx;
            yIntercept1 += gy;
            double xIntercept2 = -((b * m) + Math.sqrt(quadratic)) / (Math.pow(m, 2) + 1);
            double yIntercept2 = (m * xIntercept2) + b;
            xIntercept2 += gx;
            yIntercept2 += gy;
            if (Math.abs(two.x - xIntercept1) < Math.abs(two.x - xIntercept2)) {
                point = new Point(xIntercept1, yIntercept1);
            }
            else {
                point = new Point(xIntercept2, yIntercept2);
            }
        } 
        else {
            double pX = -b / (m + 1 / m);
            double pY = pX * m + b;
            point = new Point(pX, pY);
        }
        return point;
    }

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
