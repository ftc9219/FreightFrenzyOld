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

        ArrayList<Point> points = new ArrayList<Point>();
        Point point;
        double radius = 1;
        double gx = hardware.GlobalPos[0];
        double gy = hardware.GlobalPos[1];
        if (two.x - one.x < .0003) {
            two.x += .0003;
        }
        double m = (two.y - one.y) / (two.x - one.x);
        double y1 = one.y - gy;
        double x1 = one.x - gx;
        double b = y1 - (m * x1);

        try {

            double xIntercept1 = ((-b * m) + Math.sqrt(Math.pow(radius, 2) + (Math.pow(m, 2) * Math.pow(radius, 2)) - Math.pow(b, 2))) / (Math.pow(m, 2) + 1);
            double yIntercept1 = (m * xIntercept1) + b;
            xIntercept1 += gx;
            yIntercept1 += gy;
            double xIntercept2 = -((b * m) + Math.sqrt(Math.pow(radius, 2) + (Math.pow(m, 2) * Math.pow(radius, 2)) - Math.pow(b, 2))) / (Math.pow(m, 2) + 1);
            double yIntercept2 = (m * xIntercept2) + b;
            xIntercept2 += gx;
            yIntercept2 += gy;
            point = new Point(xIntercept1, yIntercept1);
            points.add(point);
            point = new Point(xIntercept2, yIntercept2);
            points.add(point);

        }
        catch (Exception e) {
            
        }

        return points;

    }


}
