package org.example.domino.simulation;

import org.example.domino.model.DominoBody;
import org.example.domino.model.DominoPivot;

import java.util.ArrayList;
import java.util.List;

public class DominoSimulation {

    private static final double G = 9.81;

    private static final double START_ANGLE_DEG = 0.5;
    private static final double START_OMEGA = -0.25;

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
        body.applyImpulseAtPoint(jx, jy, px, py);
        resting = false;
        active = true;

        if(jx < 0) {
            pivot.setByBR(World.GROUND_Y);
        } else {
            pivot.setByBL(World.GROUND_Y);
        }
        // DominoKinematics.updateCOMFromPivot(body, pivot);
    }

    public void giveTopLeftImpulse(double impulseMagnitude) {
        double[] world_tl = body.worldCornerBL();

        // Impuls nach rechts
        double jx = -impulseMagnitude;
        double jy = 0.0;
        giveImpulseAtPoint(jx, jy, world_tl[0], world_tl[1]);
    }
    // ===================================================================

    public void update(double dt) {
        if (resting) {
            return;
        }
        solveCorner(dt);

        if (isOnFloor()) {
            return;
        }

        updateComFromPivot();

        body.setVx(0.0);
        body.setVy(0.0);

        // --- Kollision mit dem nächsten Domino in der Kette --- todo
    }

    private boolean isOnFloor() {
        double minAngle = -Math.PI / 2.0;
        double angle = body.getAngle();
        if(angle<=minAngle) {
            body.setAngle(angle);
            pivot.setByBrTr(World.GROUND_Y);
            body.inelasticCollision();

            resting = true;
            active = false;
            return true;
        }
        return false;
    }

    public void reset(double x) {
        body.setX(x);
        body.setY(World.GROUND_Y+ body.getHeight() / 2.0);
        body.setAngle(0.0);
        body.setOmega(0.0);

        body.setVx(0.0);
        body.setVy(0.0);

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
}
