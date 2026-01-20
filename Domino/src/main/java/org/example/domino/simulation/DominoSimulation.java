package org.example.domino.simulation;

import org.example.domino.model.DominoBody;
import org.example.domino.model.DominoPivot;

public class DominoSimulation {

    private static final double G = 9.81;

    private static final double ROT_DAMPING = 0.0005;

    private final DominoBody body;
    private final DominoPivot pivot;

    private boolean resting = true;
    private boolean active = false;

    public DominoSimulation(double x_position) {
        double domino_w_default = 20.0;
        double domino_h_default = 120.0;
        double domino_mass_default = 1.0;

        this.body = new DominoBody(
                x_position,
                World.GROUND_Y+ domino_h_default / 2.0,
                domino_w_default,
                domino_h_default,
                domino_mass_default);
        this.pivot = new DominoPivot(body);
        this.pivot.setByBlBr(World.GROUND_Y);
    }


    // ==================== IMPULSÜBERTRAGUNG ====================
    // IMPULS AN BELIEBIGEM PUNKT
    // px, py → Weltkoordinaten
    public void giveImpulseAtPoint(double jx, double jy, double px, double py) {
        System.out.println("Impulse: jx=" + jx + " jy=" + jy);
        System.out.println("Punkt: px= " + px + " py=" + py);
        body.applyImpulseAtPoint(jx, jy, px, py);
        resting = false;
        active = true;
        // nur rechts
        pivot.setByBR(World.GROUND_Y);
        updateComFromPivot();
        System.out.println("Body get impuls: " + body);
    }

    public void giveTopLeftImpulse(double impulseMagnitude) {
        double[] world_tl = body.worldCornerTL();

        // Impuls nach rechts
        double jx = impulseMagnitude;
        double jy = 0.0;
        giveImpulseAtPoint(jx, jy, world_tl[0], world_tl[1]);
    }
    // ===================================================================

    public void update(double dt) {
        if (resting || !active) return;

        solveCorner(dt);

        if (isOnFloor()) return;

        updateComFromPivot();

        body.setVx(0.0);
        body.setVy(0.0);
    }

    public boolean isOnFloor() {
        double minAngle = -Math.PI / 2.0;
        double angle = body.getAngle();
        if(angle<=minAngle) {
            body.setAngle(angle);
            pivot.setByBrTr(World.GROUND_Y);
            this.inelasticCollision();

            resting = true;
            active = false;
            return true;
        }
        return false;
    }

    public void deactivate() {
        active = false;
    }

    public void reset(double x) {
        body.setX(x);
        body.setY(World.GROUND_Y+ body.getHeight() / 2.0);
        body.setAngle(0.0);
        body.setOmega(0.0);

        body.setVx(0.0);
        body.setVy(0.0);

        resting = true;
        active = false;

        pivot.setByBlBr(World.GROUND_Y);
    }

    @Override
    public String toString() {
        return "DominoSimulation{" +
                "body=" + body +
                ", pivot=" + pivot +
                ", resting=" + resting +
                ", active=" + active +
                '}';
    }

    public DominoBody getBody() {
        return body;
    }

    /** Schwerpunktkoordinaten aus Pivot + Winkel */
    public void updateComFromPivot() {
        double hw = body.getWidth() / 2.0;
        double hh = body.getHeight() / 2.0;

        double a = body.getAngle();
        double c = Math.cos(a);
        double s = Math.sin(a);

        double vx = (hw) * c - (-hh) * s;
        double vy = (hw) * s + (-hh) * c;

        body.setX(pivot.getX() - vx);
        body.setY(pivot.getY() - vy);
    }

    /** CORNER: Rotation um festen Pivot */
    public void solveCorner(double dt) {
        double angle = body.getAngle();
        double omega = body.getOmega();

        double w = body.getWidth();
        double h = body.getHeight();

        double m = body.getMass();

        double rLocalX = pivot.getLocalX();
        double rLocalY = pivot.getLocalY();

        double c = Math.cos(angle);
        double s = Math.sin(angle);

        // Abstand Schwerpunkt -> Pivot (lokal)
        double rX = rLocalX * c - rLocalY * s; // todo Prüfen ob -

        // Gravitationsmoment
        double tau = rX * (-m * G);

        // Trägheitsmoment um Pivot -
        double Icm = body.getInertia();
        double r2 = rLocalX * rLocalX + rLocalY * rLocalY;
        double Ipivot = Icm + m * r2; // Der Satz von Steiner

        // Winkelbeschleunigung
        double alphaDD = tau / Ipivot;

        // Integration
        omega += alphaDD * dt;
        omega *= (1.0 - ROT_DAMPING);
        angle += omega * dt;

        body.setOmega(omega);
        body.setAngle(angle);
    }

    public double[] collisionDetection(DominoSimulation next){
        double ax = this.getBody().getX();
        double ay = this.getBody().getY();
        double aAngle = this.getBody().getAngle();

        double bx = next.getBody().getX();
        double by = next.getBody().getY();
        double bAngle = next.getBody().getAngle();

        // ---------- cos una sin ----------
        double ca = Math.cos(aAngle);
        double sa = Math.sin(aAngle);
        double cb = Math.cos(bAngle);
        double sb = Math.sin(bAngle);

        // die normierten Achsen der lokalen Koordinaten der Rechtecke A und B
        // die zur Weltebene hin gedreht wurden.
        double[][] axes = {
                { ca,  sa},   // A x-axis
                {-sa,  ca},   // A y-axis
                { cb,  sb},   // B x-axis
                {-sb,  cb}    // B y-axis
        };

        // ---------- Corners A und B ----------
        double[][] A = this.getBody().getLocalCorner();
        double[][] B = next.getBody().getLocalCorner();

        double minOverlap = Double.POSITIVE_INFINITY;
        double bestNx = 0;
        double bestNy = 0;

        // ---------- SAT - Separating Axis Theorem ----------
        // https://dyn4j.org/2010/01/sat/
        for (double[] axis : axes) {

            double len = Math.hypot(axis[0], axis[1]);
            axis[0] /= len;
            axis[1] /= len;

            double minA = Double.POSITIVE_INFINITY;
            double maxA = Double.NEGATIVE_INFINITY;
            double minB = Double.POSITIVE_INFINITY;
            double maxB = Double.NEGATIVE_INFINITY;

            for (double[] v : A) {
                double wx = ax + v[0]*ca - v[1]*sa;
                double wy = ay + v[0]*sa + v[1]*ca;
                double p = wx*axis[0] + wy*axis[1];
                minA = Math.min(minA, p);
                maxA = Math.max(maxA, p);
            }

            for (double[] v : B) {
                double wx = bx + v[0]*cb - v[1]*sb;
                double wy = by + v[0]*sb + v[1]*cb;
                double p = wx*axis[0] + wy*axis[1];
                minB = Math.min(minB, p);
                maxB = Math.max(maxB, p);
            }

            double overlap = Math.min(maxA, maxB) - Math.max(minA, minB);

            if (overlap <= 0) return null; // keine Kollision

            if (overlap < minOverlap) {
                minOverlap = overlap;
                bestNx = axis[0];
                bestNy = axis[1];
            }
        }

        return new double[]{
                minOverlap,
                bestNx, bestNy,
        };
    }

    public void inelasticCollision() {
        body.setVx(0);
        body.setVy(0);
        body.setOmega(0);
    }

    public void resolvePenetration(DominoSimulation next, double nx, double ny, double depth) {
        DominoBody A = this.getBody();
        DominoBody B = this.getBody();
        double slop = 0.0001;

        double correction = Math.max(depth - slop, 0.0);

        A.setX(A.getX() - nx * correction * 0.5);
        A.setY(A.getY() - ny * correction * 0.5);

        B.setX(B.getX() + nx * correction * 0.5);
        B.setY(B.getY() + ny * correction * 0.5);
    }

    public double[] getCollisionPoints(DominoSimulation next) {
        DominoBody A = this.getBody();
        DominoBody B = next.getBody();

        double[] aTR = A.worldCornerTR();
        double[] bBL = B.worldCornerBL();

        if(B.containsWorldPoint(aTR[0],aTR[1])) {
            return aTR;
        } else {
            return bBL;
        }

    }
    public void clampAngleAgainst(DominoSimulation next) {
        // Wir drehen in kleinen Schritten zurück, bis keine Kollision mehr da ist.
        // Schrittgröße klein halten, sonst "springt" es sichtbar.
        final double step = 0.0015;  // ~0.086 Grad
        final int maxIter = 60;

        for (int it = 0; it < maxIter; it++) {
            double[] hit = this.collisionDetection(next);
            if (hit == null) return; // keine Überlappung mehr -> fertig

            body.setAngle(body.getAngle() + step);

            updateComFromPivot();
        }
    }

}
