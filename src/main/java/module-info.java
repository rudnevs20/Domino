module org.example.domino {
    requires javafx.controls;
    requires javafx.fxml;
    requires java.xml;
    requires java.desktop;

    exports org.example.domino.app;
    exports org.example.domino.simulation;
    exports org.example.domino.view;
    exports org.example.domino.model;
    exports org.example.domino.physics;
    opens org.example.domino.app to javafx.graphics;
}
