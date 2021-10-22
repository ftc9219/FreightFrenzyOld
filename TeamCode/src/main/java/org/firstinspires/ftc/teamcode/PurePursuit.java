package org.firstinspires.ftc.teamcode;
import org.opencv.core.Point;
import java.util.ArrayList;

public class PurePursuit {

    Hardware hardware = new Hardware();

     private ArrayList<Point> points = new ArrayList<Point>();

     private int currentLine = 0;

     private double distanceToTurn;

     private double turnAngle;

     public boolean pathDone;

     public void clearPoints() {
         points.clear();
         currentLine = 0;
     }

     public void addPoint(double x, double y) {
         Point point = new Point(x, y);
         points.add(point);
     }

     public void followPath() {

        Point point;
        Point one = points.get(currentLine);
        Point two = points.get(currentLine + 1);
        double radius = 1;

        if (currentLine < points.size()) {
            pathDone = false;
        }
        else {
            pathDone = true;
        }

        if (!pathDone) {
            double gx = hardware.GlobalPos[0];
            double gy = hardware.GlobalPos[1];
            double d = Math.sqrt(Math.pow(gy - two.y, 2) + Math.pow(gx - two.x, 2));
            if (d < 3) {
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
                    double m2 = -1 / m;
                    double pX = b / (m2 - m);
                    double pY = ((pX * m) + b) + gy;
                    pX += gx;
                    point = new Point(pX, pY);
                    distanceToTurn = Math.hypot((gx - point.x), (gy - point.y));
                }
            }
            else {
                currentLine++;
            }
        }
    }


}


