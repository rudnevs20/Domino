package org.example.domino.simulation;

import org.example.domino.model.DominoBody;

import java.util.ArrayList;
import java.util.List;

public class World {

    private static final double G = 9.81;

    public static final double GROUND_Y = 60.0;
    public static final double GROUND_X = 300.0;

    private static final double START_ANGLE_DEG = 0.5;
    private static final double START_OMEGA = -0.25;
    private static final double ABSTAND = 60.0;

    private static final double ROT_DAMPING = 0.0005;

    //private List<DominoBody> dominos = new ArrayList<>();
    private List<DominoSimulation> dominoSimulations = new ArrayList<>();

    private boolean running = false;
    private boolean started = false;

    private int active = 0;

    private double pivotX;
    private double pivotY;

    public World() {
        double w = 20.0;
        double h = 120.0;

        for (int i = 0; i < 10; i++) {
            double x = GROUND_X + i*ABSTAND;
            double y = GROUND_Y +h / 2.0;
            //dominos.add(new DominoBody(x, y, w, h, 1.0));

            dominoSimulations.add(new DominoSimulation(x));
        }
//        reset();
    }

//    public List<DominoBody> getDominos() {
//        return dominos;
//    }
    public List<DominoSimulation> getDominoSimulations() {
        return dominoSimulations;
    }

    public void startSimulation() {
        if (started) return;

        started = true;
        active = 0;
        running = true;

//        DominoBody d = dominos.get(0);
        DominoSimulation dS = dominoSimulations.get(0);


        dS.giveTopLeftImpulse(5);
//        pivotX = d.getX() + d.getWidth() / 2.0;
//        pivotY = GROUND_Y;
//        d.setAngle(Math.toRadians(-START_ANGLE_DEG));
//        d.setOmega(START_OMEGA);
        //---

         dS.updateComFromPivot();
//        updateComFromPivot(d);
    }

    public void reset() {
        running = false;
        started = false;

//        for (int i = 0; i < dominos.size(); i++) {
//            DominoBody d = dominos.get(i);
//
//            double h = d.getHeight();
//
//            double x = GROUND_X + i * 60.0; //einstellbarer Abstand zwischen Dominos
//            double y = GROUND_Y + h / 2.0;
//
//            d.setX(x);
//            d.setY(y);
//            d.setAngle(0.0);
//            d.setOmega(0.0);
//
//            d.setVx(0.0);
//            d.setVy(0.0);
//        }
//        DominoBody first = dominos.get(0);
//        pivotX = first.getX() + first.getWidth() / 2.0;
//        pivotY = GROUND_Y;
//        active = 0;

        for(int i = 0; i < dominoSimulations.size(); i++){
            DominoSimulation dS = dominoSimulations.get(i);
            double x = GROUND_X + i * 60.0; //einstellbarer Abstand zwischen Dominos
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
        DominoBody d = dominoSimulations.get(active).getBody();   // <-- DER DOMINO, DER SIMULIERT WIRD
//      DominoBody d = dominos.get(active);
//        double angle = d.getAngle();
//        double omega = d.getOmega();
//
//        double minAngle = -Math.PI / 2.0;
//
//        double w = d.getWidth();
//        double h = d.getHeight();
//
//        double rLocalX = -w / 2.0;
//        double rLocalY =  h / 2.0;
//
//        double c = Math.cos(angle);
//        double s = Math.sin(angle);
//
//        double rX = rLocalX * c - rLocalY * s;
//
//        double tau = rX * (-d.getMass() * G);
//
//        double r2 = rLocalX * rLocalX + rLocalY * rLocalY;
//        double I = d.getInertia() + d.getMass() * r2;
//
//        double alpha = tau / I;
//
//        omega += alpha * dt;
//        omega *= (1.0 - ROT_DAMPING);
//
//        angle += omega * dt;
//
//        if (angle <= minAngle) {
//            angle = minAngle;
//            omega = 0.0;
//            d.setAngle(angle);
//            d.setOmega(omega);
//
//            updateComFromPivot(d);
//            running = false;
//            return;
//        }
//
//        d.setAngle(angle);
//        d.setOmega(omega);
//
//        updateComFromPivot(d);
//
//        d.setVx(0.0);
//        d.setVy(0.0);

//         --- Kollision mit dem nächsten Domino in der Kette ---
        if (active + 1 < dominoSimulations.size()) {

            DominoBody next = dominoSimulations.get(active + 1).getBody();

            // vordere obere Ecke vom aktiven Domino
            double frontX = d.worldPointX(+d.getWidth()/2.0, +d.getHeight()/2.0);
            double nextLeft = next.getX() - next.getWidth()/2.0;

            if (frontX >= nextLeft) {

                // (A) Clamp: ein kleines Stück zurück, damit er nicht "durchdringt"
                int it = 0;
                while (frontX > nextLeft && it < 30) {
                    double angle = d.getAngle();
                    angle += 0.0015;          // WICHTIG: bei dir meist + statt -
                    d.setAngle(angle);
                    dominoSimulations.get(active).updateComFromPivot();
//                    updateComFromPivot(d);
                    frontX = d.worldPointX(+d.getWidth()/2.0, +d.getHeight()/2.0);
                    it++;
                }

                // (B) Energie-basierte Übergabe: p = Anteil der Rotationsenergie, die weitergeht
                double p = 0.8; // 80% Energie weiter, 20% Verlust
                double wOld = d.getOmega();

                double wNext = Math.copySign(Math.sqrt(p) * Math.abs(wOld), wOld);
                dominoSimulations.get(active+1).giveTopLeftImpulse(1);
                next.setAngle(Math.toRadians(-0.2)); // kleiner Startkipper
                next.setOmega(wNext);

                // alter Domino verliert seinen Schwung
                d.inelasticCollision();
//                d.setOmega(0.0);

                // (C) nächster Domino wird aktiv, Pivot neu setzen
                active++;
                dominoSimulations.get(active).updateComFromPivot();
//                pivotX = next.getX() + next.getWidth() / 2.0;
//                pivotY = GROUND_Y;
//                updateComFromPivot(next);
            }
        }

    }


//    private void updateComFromPivot(DominoBody d) {
//        double hw = d.getWidth() / 2.0;
//        double hh = d.getHeight() / 2.0;
//
//        double a = d.getAngle();
//        double c = Math.cos(a);
//        double s = Math.sin(a);
//
//        double vx = (hw) * c - (-hh) * s;
//        double vy = (hw) * s + (-hh) * c;
//
//        d.setX(pivotX - vx);
//        d.setY(pivotY - vy);
//    }
}
