package org.example.domino.app;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.control.Button;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.stage.Stage;
import org.example.domino.simulation.World;
import org.example.domino.view.WorldView;

public class MainApp extends Application {

    private static final int WIDTH = 1000;
    private static final int HEIGHT = 600;

    @Override
    public void start(Stage stage) {

        Canvas canvas = new Canvas(WIDTH, HEIGHT);

        World world = new World();
        WorldView view = new WorldView(canvas, world);

        Button startBtn = new Button("Start");
        Button resetBtn = new Button("Reset");

        startBtn.setOnAction(e -> world.startSimulation());
        resetBtn.setOnAction(e -> world.reset());

        HBox controls = new HBox(10, startBtn, resetBtn);

        BorderPane root = new BorderPane();
        root.setTop(controls);
        root.setCenter(canvas);

        Scene scene = new Scene(root, WIDTH, HEIGHT);
        stage.setTitle("Domino Simulation");
        stage.setScene(scene);
        stage.show();

        AnimationTimer timer = new AnimationTimer() {
            private long last = 0;

            @Override
            public void handle(long now) {
                if (last == 0) { last = now; return; }

                double dt = (now - last) / 1e9;
                last = now;

                dt = Math.min(dt, 0.033);

                world.update(dt);
                view.render();
            }
        };
        timer.start();
    }

    public static void main(String[] args) {
        launch(args);
    }
}
