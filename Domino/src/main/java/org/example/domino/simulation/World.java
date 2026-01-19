package org.example.domino.simulation;

import org.example.domino.model.DominoBody;

import java.util.ArrayList;
import java.util.List;

public class World {

    public static final double GROUND_Y = 60.0;
    public static final double GROUND_X = 300.0;

    private static final double ABSTAND = 60.0;

    private List<DominoSimulation> dominoSimulations = new ArrayList<>();

    private boolean running = false;
    private boolean started = false;

    public World() {

        for (int i = 0; i < 10; i++) {
            double x = GROUND_X + i*ABSTAND;
            dominoSimulations.add(new DominoSimulation(x));
        }
    }

    public List<DominoSimulation> getDominoSimulations() {
        return dominoSimulations;
    }

    public void startSimulation() {
        if (started) return;

        started = true;
        running = true;

        DominoSimulation dS = dominoSimulations.get(0);

        dS.giveTopLeftImpulse(2);
    }

    public void reset() {
        running = false;
        started = false;

        for(int i = 0; i < dominoSimulations.size(); i++){
            DominoSimulation dS = dominoSimulations.get(i);
            double x = GROUND_X + i * ABSTAND; //einstellbarer Abstand zwischen Dominos
            dS.reset(x);
        }
    }

    public void update(double dt) {
        if (!running) return;

        dt = Math.min(dt, 0.02);
        dt *= 3.5;

        for (DominoSimulation dS : dominoSimulations) {
            dS.update(dt);
        }

//        --- Kollision für alle Dominos ---
        for(int i = 0; i < dominoSimulations.size() - 1; i++){
//            --- Kollision mit dem nächsten Domino in der Kette
            DominoSimulation before = dominoSimulations.get(i);
            DominoSimulation next = dominoSimulations.get(i+1);
            double[] hit = before.collisionDetection(next);

            if (hit != null) {
                DominoBody A = before.getBody();
                DominoBody B = next.getBody();

                double penetration = hit[0]; // penetration depth
                double nx = hit[1]; // collision normal
                double ny = hit[2]; // collision normal

                // Einfachere Version TR of before oder BL of next
                double[] collisionPoint = before.getCollisionPoints(next);
                if(collisionPoint == null) return;
                double cx = collisionPoint[0]; // collision point
                double cy = collisionPoint[1]; // collision point

                // ---------- die Geschwindigkeit des Aufprallpunkts bei A ----------
                double vAx = A.velAtPointX(cx, cy);
                double vAy = A.velAtPointY(cx, cy);

                // ---------- Projektion der Geschwindigkeit auf die Normale ----------
                double vRel = vAx * nx + vAy * ny;

                System.out.println("vAx: " + vAx + " vAy: " + vAy + " vRel: " + vRel + " nx=" + nx + " ny=" + ny + " overlap: " + penetration);
                System.out.println("Punkt: cx= " + cx + " cy=" + cy);

                // (A) Clamp: ein kleines Stück zurück, damit er nicht "durchdringt"
                before.resolvePenetration(nx, ny, penetration);

                // Ist das positiv, hat bereits eine Kollision stattgefunden;
                // ist es negativ, steht ein Dominoeffekt kurz bevor
                if (vRel <= 0) return;
                // Wenn der Auftreffpunkt unterhalb der Mitte liegt,
                // erfolgt die Rotation von unten nach oben
                if(cy<=B.getY()) return;


                // (B) Energie-basierte Übergabe: p = Anteil der Rotationsenergie, die weitergeht
                double p = 0.8; // 80% Energie weiter, 20% Verlust
                double j = A.getMass() * Math.abs(vRel);
                j *= p;

                double jx = j * nx;
                double jy = j * ny;

                System.out.println("Before: " + A);
                // alter Domino verliert seinen Schwung
                before.inelasticCollision();

                // (C) nächster Domino wird aktiv, kriegt Impuls, Pivot neu setzen
//                next.giveTopLeftImpulse(jx);
                System.out.println("Normal: nx=" + nx + " ny=" + ny);
                System.out.println("Impulse: jx=" + jx + " jy=" + jy);
                next.giveImpulseAtPoint(jx, jy, cx, cy);

            }
        }
    }



}
