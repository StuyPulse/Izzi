package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class Line {
        public double slope;
        public double yIntercept;

        public Line(double slope, double yIntercept) {
            this.slope = slope;
            this.yIntercept = yIntercept;
        }

        public Line(Translation2d point1, Translation2d point2) {
            this.slope = (point1.getY() - point2.getY()) / (point1.getX() - point2.getX());
            this.yIntercept = point1.getY() - slope * point1.getX();
        }

        public double eval(double x) {
            return slope * x + yIntercept;
        }

        public boolean under(double x, double y) {
            return y < eval(x);
        }

        public boolean above(double x, double y) {
            return y > eval(x);
        }
    }