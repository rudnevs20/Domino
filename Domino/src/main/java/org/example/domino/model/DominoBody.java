package org.example.domino.model;

/**
 * Domino als Rechteck.
 * Schwerpunkt = geometrische Mitte.
 */
public class DominoBody extends RigidBody {

    private final double width;
    private final double height;
    private final double[][] localCorner; // BL -> BR -> TR -> TL

    public DominoBody(double x, double y, double width, double height, double mass) {
        this.x = x;
        this.y = y;

        this.width = width;
        this.height = height;

        this.mass = mass;
        this.invMass = (mass > 0) ? 1.0 / mass : 0.0;

        // Trägheitsmoment um den Schwerpunkt
        this.inertia = (1.0 / 12.0) * mass * (width * width + height * height);
        this.invInertia = (inertia > 0) ? 1.0 / inertia : 0.0;

        this.angle = 0.0;
        this.vx = 0.0;
        this.vy = 0.0;
        this.omega = 0.0;

        // local coordinates of corners
        this.localCorner = new double[][]{
                {-width / 2, -height / 2}, // BL
                {width / 2, -height / 2}, // BR
                {width / 2, height / 2}, // TR
                {-width / 2, height / 2}  // TL
        };
    }

    public double getWidth() { return width; }
    public double getHeight() { return height; }

    // // ==================== World point ===============================================
    public double worldPointX(double localX, double localY) {
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        return this.x + (localX * c - localY * s);
    }
    public double worldPointY(double localX, double localY) {
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        return this.y + (localX * s + localY * c);
    }
    // ===================================================================================
    // ==================== World point of 4 corners (bl, br, tr, tl) Körpergrenze =======
    // ===================================================================================
    public double[] worldCornerBL() {
        double lx = this.localCorner[0][0];
        double ly = this.localCorner[0][1];
        return new double[]{
                worldPointX(lx, ly),
                worldPointY(lx, ly)
        };
    }
    public double[] worldCornerBR() {
        double lx = this.localCorner[1][0];
        double ly = this.localCorner[1][1];
        return new double[]{
                worldPointX(lx, ly),
                worldPointY(lx, ly)
        };
    }
    public double[] worldCornerTR() {
        double lx = this.localCorner[2][0];
        double ly = this.localCorner[2][1];
        return new double[]{
                worldPointX(lx, ly),
                worldPointY(lx, ly)
        };
    }
    public double[] worldCornerTL() {
        double lx = this.localCorner[3][0];
        double ly = this.localCorner[3][1];
        return new double[]{
                worldPointX(lx, ly),
                worldPointY(lx, ly)
        };
    }
    // ===================================================================================
    // ===================================================================================
}
