package org.example.domino.model;

/**
 * Klasse für einen starren Körper (2D).
 * Physik-Koordinaten: y nach oben positiv.
 */
public class RigidBody {

    protected double x;
    protected double y;
    protected double angle;

    protected double vx;
    protected double vy;
    protected double omega;

    protected double mass;
    protected double invMass;

    protected double inertia;
    protected double invInertia;

    // ===== Impuls- und Punktgeschwindigkeit (für Kontakt/Physik) =====

    //Geschwindigkeit eines Punkts am Körper
    public double velAtPointX(double px, double py) {
        double rx = px - x;
        double ry = py - y;
        return vx - omega * ry;
    }

    public double velAtPointY(double px, double py) {
        double rx = px - x;
        return vy + omega * rx;
    }

    // Impuls am Schwerpunkt
    public void applyImpulse(double jx, double jy) {
        vx += jx * invMass;
        vy += jy * invMass;
    }

    public void applyImpulseAtPoint(double jx, double jy, double px, double py) {
        applyImpulse(jx, jy);

        double rx = px - x;
        double ry = py - y;

        // r x J
        double cross = rx * jy - ry * jx;
        omega += cross * invInertia;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getAngle() { return angle; }

    public double getVx() { return vx; }
    public double getVy() { return vy; }
    public double getOmega() { return omega; }

    public double getMass() { return mass; }
    public double getInvMass() { return invMass; }

    public double getInertia() { return inertia; }
    public double getInvInertia() { return invInertia; }

    public void setX(double x) { this.x = x; }
    public void setY(double y) { this.y = y; }
    public void setAngle(double angle) { this.angle = angle; }

    public void setVx(double vx) { this.vx = vx; }
    public void setVy(double vy) { this.vy = vy; }
    public void setOmega(double omega) { this.omega = omega; }
}
