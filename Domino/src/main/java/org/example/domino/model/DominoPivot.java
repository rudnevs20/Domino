package org.example.domino.model;

/*
    Diese Klasse stellt den Pivot eines Dominosteins dar.
    Der Pivot kann sich entweder an einer der Ecken des Dominosteins befinden
    (Bottom Left, Bottom Right, Top Left, Top Right)
    oder an einer der Kanten, was bedeutet, dass der Dominostein flach aufliegt.
    Zum aktuellen Zeitpunkt werden nur:
        - Ecke: BL, BR
        - Kanten: BL-BR, BR-TR
 */
public class DominoPivot {
    public enum PivotType {
        NONE,
        EDGE,
        CORNER
    }

    private double x;
    private double y;

    private final DominoBody body;

    private PivotType type = PivotType.NONE;

    public DominoPivot(DominoBody body) {
        this.body = body;
    }

    public void setByBR(double floorY) {
        double[] br = body.worldCornerBR();
        x = br[0];
        y = floorY;
        type = PivotType.CORNER;
    }
    public void setByBL(double floorY) {
        double[] bl = body.worldCornerBL();
        x = bl[0];
        y = floorY;
        type = PivotType.CORNER;
    }
    public void setByBlBr(double floorY) {
        double[] bl = body.worldCornerBL();
        double[] br = body.worldCornerBL();

        x = (bl[0] + br[0]) * 0.5;
        y = floorY;
        type = PivotType.EDGE;
    }
    public void setByBrTr(double floorY) {
        double[] tr = body.worldCornerTR();
        double[] br = body.worldCornerBL();

        x = (tr[0] + br[0]) * 0.5;
        y = floorY;
        type = PivotType.EDGE;
    }

    public double getX() {
        return x;
    }
    public double getLocalX() {
        double cx = body.getX();
        double px = x;
        return cx - px;
    }

    public double getY() {
        return y;
    }

    public double getLocalY() {
        double cy = body.getY();
        double py = y;
        return cy - py;
    }

    public PivotType getType() {
        return type;
    }

    @Override
    public String toString() {
        return "DominoPivot{" +
                "pivotX=" + x +
                ", pivotY=" + y +
                ", type=" + type +
                '}';
    }
}

