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
        double speed = getSpeed();
        Point one = points.get(currentLine);
        Point two = points.get(currentLine + 1);
        double radius = 1;

        if (currentLine < points.size()) {
            pathDone = false;
            return;
        }
        else {
            pathDone = true;
        }

        if (!pathDone) {
            double gx = hardware.GlobalPos[0];
            double gy = hardware.GlobalPos[1];
            double d = Math.sqrt(Math.pow(gy - two.y, 2) + Math.pow(gx - two.x, 2));
            if (d < 3) {
                point = MathFunctions.lineCircleIntercept(one, two, gx, gy, radius);
                hardware.moveToPosition(point.x, point.y, .5, speed, 0);
            }
            else {
                currentLine++;
            }
        }
     }

     public double getSpeed() {

        double speed = 1;



        return speed;

     }


}


