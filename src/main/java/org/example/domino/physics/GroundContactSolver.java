package org.example.domino.physics;

import org.example.domino.model.DominoBody;


public class GroundContactSolver {

    private GroundContactSolver() {}

    public static void solve(DominoBody d, double groundY, double restitutionE, double mu) {

        double hw = d.getWidth() / 2.0;
        double hh = d.getHeight() / 2.0;

        solveCorner(d, -hw, -hh, groundY, restitutionE, mu);
        solveCorner(d, +hw, -hh, groundY, restitutionE, mu);
    }

    private static void solveCorner(DominoBody d,
                                    double lx, double ly,
                                    double groundY,
                                    double e, double mu) {

        double px = d.worldPointX(lx, ly);
        double py = d.worldPointY(lx, ly);

        double penetration = groundY - py;
        double contactEps = 0.002;

        boolean inContact = (penetration > 0.0) || (py <= groundY + contactEps);
        if (!inContact) return;


        double nx = 0.0;
        double ny = 1.0;

        if (penetration > 0.0) {
            double beta = 0.6;
            d.setY(d.getY() + penetration * beta);
        }

        double vpx = d.velAtPointX(px, py);
        double vpy = d.velAtPointY(px, py);
        double vn = vpx * nx + vpy * ny;

        double vnEps = 0.05;
        if (Math.abs(vn) < vnEps) {
            return;
        }

        if (vn >= 0.0) return;

        double rx = px - d.getX();
        double ry = py - d.getY();

        double rCrossN = rx * ny - ry * nx;
        double kN = d.getInvMass() + (rCrossN * rCrossN) * d.getInvInertia();
        if (kN <= 0.0) return;

        double jn = -(1.0 + e) * vn / kN;
        d.applyImpulseAtPoint(jn * nx, jn * ny, px, py);

        double tx = 1.0;
        double ty = 0.0;

        vpx = d.velAtPointX(px, py);
        vpy = d.velAtPointY(px, py);
        double vt = vpx * tx + vpy * ty;

        double vtEps = 0.02;
        if (Math.abs(vt) < vtEps) return;

        double rCrossT = rx * ty - ry * tx;
        double kT = d.getInvMass() + (rCrossT * rCrossT) * d.getInvInertia();
        if (kT <= 0.0) return;

        double jt = -vt / kT;

        double maxF = mu * Math.abs(jn);
        if (jt > maxF) jt = maxF;
        if (jt < -maxF) jt = -maxF;

        d.applyImpulseAtPoint(jt * tx, jt * ty, px, py);
    }
}
