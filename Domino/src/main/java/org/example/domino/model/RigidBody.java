package org.example.domino.model;

/**
 * Klasse für einen starren Körper (2D).
 * Physik-Koordinaten: y nach oben positiv.
 */
public class RigidBody {
    // Schwerpunkt
    protected double x;
    protected double y;
    protected double angle;
    // Geschwindigkeit
    protected double vx;
    protected double vy;
    // Winkelgeschwindigkeit
    // omega > 0 gegen den Uhrzeigersinn
    // omega < 0 im Uhrzeigersinn
    protected double omega;

    protected double mass;
    protected double invMass;// m^-1
    // Trägheitsmoment
    protected double inertia;
    protected double invInertia;// J^-1

    // ===== Impuls- und Punktgeschwindigkeit (für Kontakt/Physik) =====

    //Geschwindigkeit eines Punkts am Körper
    public double[] velAtPointXY(double px, double py) {
        return new double[]{velAtPointX(px, py), velAtPointY(px, py)};
    }
    public double velAtPointX(double px, double py) {
        double ry = py - y;
        return vx - omega * ry *0.25;
    }
    public double velAtPointY(double px, double py) {
        double rx = px - x;
        return vy + omega * rx*0.25;
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

    @Override
    public String toString() {
        return "RigidBody{" +
                "x=" + x +
                ", y=" + y +
                ", angle=" + angle +
                ", vx=" + vx +
                ", vy=" + vy +
                ", omega=" + omega +
                '}';
    }
}
