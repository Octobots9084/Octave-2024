package frc.robot.util;

public class Triangle {

    public Point point1;
    public Point point2;
    public Point point3;

    public Triangle(Point point1, Point point2, Point point3) {
        this.point1 = point1;
        this.point2 = point2;
        this.point3 = point3;
    }

    public double area() {
        return _area(point1, point2, point3);
    }

    private static double _area(Point p1, Point p2, Point p3) {
        return Math.abs(0.5 * (p1.x * (p2.y - p3.y) +
                p2.x * (p3.y - p1.y) +
                p3.x * (p1.y - p2.y)));
    }
      
    /** Checks if a point lies inside the triangle */
    public boolean isPointInside(Point testPoint) {
        // Calculate area of the whole triangle
        double totalArea = this.area();

        // Calculate areas of sub-triangles formed by connecting the test point with each vertex
        double area1 = _area(point2, point3, testPoint);
        double area2 = _area(point3, point1, testPoint);
        double area3 = _area(point1, point2, testPoint);

        // Check if the sum of sub-triangle areas is (almost) equal to the total area.
        // A small tolerance is added to account for floating-point precision errors.
        final double epsilon = 0.01;
        return Math.abs(totalArea - (area1 + area2 + area3)) < epsilon;
    }

    @Override
    public String toString() {
        return "Triangle (" + point1 + ", " + point2 + ", " + point3 + ")";
    }
}
