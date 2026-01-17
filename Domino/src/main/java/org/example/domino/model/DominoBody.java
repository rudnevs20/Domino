package org.example.domino.model;

/**
 * Domino als Rechteck.
 * Schwerpunkt = geometrische Mitte.
 */
public class DominoBody extends RigidBody {

    private final double width;
    private final double height;

    public DominoBody(double x, double y, double width, double height, double mass) {
        this.x = x;
        this.y = y;

        this.width = width;
        this.height = height;

        this.mass = mass;
        this.invMass = (mass > 0) ? 1.0 / mass : 0.0;

        // Trägheitsmoment um den Schwerpunkt
        this.inertia = (1.0 / 12.0) * mass * (width * width + height * height);
        this.invInertia = (inertia > 0) ? 1.0 / inertia : 0.0;

        this.angle = 0.0;
        this.vx = 0.0;
        this.vy = 0.0;
        this.omega = 0.0;
    }

    public double getWidth() { return width; }
    public double getHeight() { return height; }

    public double worldPointX(double localX, double localY) {
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        return x + (localX * c - localY * s);
    }

    public double worldPointY(double localX, double localY) {
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        return y + (localX * s + localY * c);
    }
}
