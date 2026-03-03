package org.example.domino.app;

import javafx.animation.AnimationTimer;
import javafx.application.Application;

import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.TextField;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.stage.Stage;
import org.example.domino.simulation.World;
import org.example.domino.view.WorldView;

import java.awt.*;

public class MainApp extends Application {

    private static final int WIDTH_WINDOW = 1000;
    private static final int MIN_ABSTAND = 33;
    private static final int MAX_ABSTAND = 111;
    private static final int WIDTH_CANVAS = (int) (2 * World.GROUND_X + 9 * MAX_ABSTAND);
    private static final int HEIGHT = 600;



    @Override
    public void start(Stage primaryStage) {


        Canvas canvas = new Canvas();
        canvas.setHeight(HEIGHT);
        canvas.setWidth(WIDTH_CANVAS);
        World world = new World();
        WorldView view = new WorldView(canvas, world);

        ScrollPane scrollPane = new ScrollPane();
        scrollPane.setContent(canvas);
        scrollPane.fitToWidthProperty().set(true);
        scrollPane.hbarPolicyProperty().setValue(ScrollPane.ScrollBarPolicy.ALWAYS);
        scrollPane.vbarPolicyProperty().setValue(ScrollPane.ScrollBarPolicy.NEVER);


        Button startBtn = new Button("Start");
        Button resetBtn = new Button("Reset");
        Label label = new Label("Abstand:");


        startBtn.setOnAction(e -> world.startSimulation());
        resetBtn.setOnAction(e -> world.reset());

        TextField textField = new TextField(world.stringAbstand);

        textField.textProperty().addListener((observable, wertAlt, eingabe) -> {
            if (eingabe.matches("\\d*")) { // Nur Zahlen erlauben
                if (MIN_ABSTAND <= Integer.parseInt(eingabe)){
                    if (MAX_ABSTAND >= Integer.parseInt(eingabe)){
                        world.stringAbstand = eingabe;
                    }
                }
            } else {
                textField.setText(wertAlt); // Ungültige Eingabe rückgängig machen
            }
        });

        HBox controls = new HBox(10, startBtn, resetBtn, label, textField);

        BorderPane root = new BorderPane();
        root.setTop(controls);
        root.setCenter(canvas);
        root.setBottom(scrollPane);

        Scene scene = new Scene(root, WIDTH_WINDOW, HEIGHT);
        primaryStage.setTitle("Domino Simulation");
        primaryStage.setScene(scene);
        primaryStage.show();

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
