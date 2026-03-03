package org.example.domino.physics;

import org.example.domino.model.DominoBody;

public class DominoContactSolver {

    private DominoContactSolver() {}

    public static void solve(DominoBody a, DominoBody b,
                             double px, double py,
                             double nx, double ny,
                             double restitutionE) {

        // Geschwindigkeit am Kontaktpunkt
        double vax = a.velAtPointX(px, py);
        double vay = a.velAtPointY(px, py);
        double vbx = b.velAtPointX(px, py);
        double vby = b.velAtPointY(px, py);

        double rvx = vax - vbx;
        double rvy = vay - vby;

        double vn = rvx * nx + rvy * ny;
        if (vn >= 0.0) return; // drückt nicht rein

        double rax = px - a.getX();
        double ray = py - a.getY();
        double rbx = px - b.getX();
        double rby = py - b.getY();

        double rAcrossN = rax * ny - ray * nx;
        double rBcrossN = rbx * ny - rby * nx;

        double k = a.getInvMass() + b.getInvMass()
                + (rAcrossN * rAcrossN) * a.getInvInertia()
                + (rBcrossN * rBcrossN) * b.getInvInertia();

        if (k <= 0.0) return;

        double j = -(1.0 + restitutionE) * vn / k;

        double jx = j * nx;
        double jy = j * ny;

        // gleicher Impuls, entgegengesetzt
        a.applyImpulseAtPoint(-jx, -jy, px, py);
        b.applyImpulseAtPoint(+jx, +jy, px, py);
    }
}
