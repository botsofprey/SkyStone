package Autonomous;

public class Rectangle extends Shape {

    public Rectangle() { super(); }
    public Rectangle(int x, int y, int width, int height) { super(x, y, width, height); }

    @Override
    public double getArea() {
        return width * height;
    }
}
