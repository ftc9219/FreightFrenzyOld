package org.firstinspires.ftc.teamcode;
import org.opencv.core.Point;
import java.util.ArrayList;
import java.util.List;

public class PurePursuit {

     private List<Point> points = new ArrayList<Point>();

     public void clearPoints() {
         points.clear();
     }

     public void addPoint(double x, double y) {
         Point point = new Point(x, y);
         points.add(point);
     }
     

}


