package org.firstinspires.ftc.teamcode;
import org.opencv.core.Point;
import java.util.ArrayList;

public class PurePursuit {

    Hardware hardware = new Hardware();

     private ArrayList<Point> points = new ArrayList<Point>();

     public void clearPoints() {
         points.clear();
     }

     public void addPoint(double x, double y) {
         Point point = new Point(x, y);
         points.add(point);
     }

     public void followPath() {

         double x = hardware.GlobalPos[0];
         double y = hardware.GlobalPos[1];
         double a = hardware.GlobalPos[2];
         double acceleration = 1;


     }


}


