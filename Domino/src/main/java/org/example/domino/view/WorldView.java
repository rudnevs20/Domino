package org.example.domino.view;

import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import org.example.domino.model.DominoBody;
import org.example.domino.simulation.World;


public class WorldView {

    private static final boolean SHOW_COM = true;

    private final Canvas canvas;
    private final GraphicsContext gc;
    private final World world;

    public WorldView(Canvas canvas, World world) {
        this.canvas = canvas;
        this.gc = canvas.getGraphicsContext2D();
        this.world = world;
    }

    public void render() {
        gc.setFill(Color.WHITE);
        gc.fillRect(0, 0, canvas.getWidth(), canvas.getHeight());

        drawGround();
        for (DominoBody d : world.getDominos()){
            drawDomino(d);
            if (SHOW_COM) drawCOM(d);
        }
    }

    private void drawGround() {
        gc.setStroke(Color.DARKGRAY);
        gc.setLineWidth(2);

        double yScreen = canvas.getHeight() - World.GROUND_Y;
        gc.strokeLine(0, yScreen, canvas.getWidth(), yScreen);
    }

    private void drawDomino(DominoBody d) {
        gc.save();

        double sx = d.getX();
        double sy = canvas.getHeight() - d.getY();

        gc.translate(sx, sy);
        gc.scale(1, -1); // jetzt ist y nach oben positiv
        gc.rotate(Math.toDegrees(d.getAngle()));

        gc.setFill(Color.GRAY);
        gc.fillRect(
                -d.getWidth() / 2.0,
                -d.getHeight() / 2.0,
                d.getWidth(),
                d.getHeight()
        );

        gc.restore();
    }

    private void drawCOM(DominoBody d) {
        double sx = d.getX();
        double sy = canvas.getHeight() - d.getY();

        gc.setFill(Color.DODGERBLUE);
        gc.fillOval(sx - 4, sy - 4, 8, 8);
    }
}
