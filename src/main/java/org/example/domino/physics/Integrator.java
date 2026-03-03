package org.example.domino.physics;

import org.example.domino.model.RigidBody;

public class Integrator {

    private Integrator() {}

    public static void step(RigidBody b, double dt) {

        b.setX(b.getX() + b.getVx() * dt);
        b.setY(b.getY() + b.getVy() * dt);

        b.setAngle(b.getAngle() + b.getOmega() * dt);
    }
}