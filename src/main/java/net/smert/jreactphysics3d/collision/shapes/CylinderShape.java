package net.smert.jreactphysics3d.collision.shapes;

import net.smert.jreactphysics3d.configuration.Defaults;
import net.smert.jreactphysics3d.mathematics.Mathematics;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a cylinder collision shape around the Y axis and centered at the origin. The cylinder is
 * defined by its height and the radius of its base. The "transform" of the corresponding rigid body gives an
 * orientation and a position to the cylinder. This collision shape uses an extra margin distance around it for
 * collision detection purpose. The default margin is 4cm (if your units are meters, which is recommended). In case, you
 * want to simulate small objects (smaller than the margin distance), you might want to reduce the margin by specifying
 * your own margin distance using the "margin" parameter in the constructor of the cylinder shape. Otherwise, it is
 * recommended to use the default margin distance by not using the "margin" parameter in the constructor.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class CylinderShape extends CollisionShape {

    // Half height of the cylinder
    private final float halfHeight;

    // Radius of the base
    private final float radius;

    // Constructor
    public CylinderShape(float radius, float height, float margin) {
        super(CollisionShapeType.CYLINDER, margin);
        assert (height > 0.0f);
        assert (radius > 0.0f);
        halfHeight = height * 0.5f;
        this.radius = radius;
    }

    // Copy-constructor
    public CylinderShape(CylinderShape shape) {
        super(shape);
        halfHeight = shape.halfHeight;
        radius = shape.radius;
    }

    // Return the radius
    public float getRadius() {
        return radius;
    }

    // Return the height
    public float getHeight() {
        return halfHeight + halfHeight;
    }

    // Test equality between two cylinder shapes
    @Override
    public boolean isEqualTo(CollisionShape otherCollisionShape) {
        CylinderShape otherShape = (CylinderShape) otherCollisionShape;
        return (radius == otherShape.radius && halfHeight == otherShape.halfHeight);
    }

    // Return a local support point in a given direction with the object margin
    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction, Vector3 supportPoint) {

        // Compute the support point without the margin
        getLocalSupportPointWithoutMargin(direction, supportPoint);

        // Add the margin to the support point
        Vector3 unitDirection = new Vector3(0.0f, 1.0f, 0.0f);
        if (direction.lengthSquare() >= Defaults.MACHINE_EPSILON * Defaults.MACHINE_EPSILON) {
            unitDirection.set(direction).normalize();
        }

        return supportPoint.add(unitDirection.multiply(margin));
    }

    // Return a local support point in a given direction without the object margin
    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction, Vector3 supportPoint) {

        float uDotv = direction.getY();
        Vector3 w = new Vector3(direction.getX(), 0.0f, direction.getZ());
        float lengthW = Mathematics.Sqrt(direction.getX() * direction.getX() + direction.getZ() * direction.getZ());

        if (lengthW > Defaults.MACHINE_EPSILON) {
            if (uDotv < 0.0f) {
                supportPoint.setY(-halfHeight);
            } else {
                supportPoint.setY(halfHeight);
            }
            supportPoint.add(w.multiply(radius / lengthW));
        } else {
            if (uDotv < 0.0f) {
                supportPoint.setY(-halfHeight);
            } else {
                supportPoint.setY(halfHeight);
            }
        }

        return supportPoint;
    }

    // Return the local inertia tensor of the cylinder
    @Override
    public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
        float height = 2.0f * halfHeight;
        float diag = (1.0f / 12.0f) * mass * (3 * radius * radius + height * height);
        tensor.set(diag, 0.0f, 0.0f,
                0.0f, 0.5f * mass * radius * radius, 0.0f,
                0.0f, 0.0f, diag);
    }

    // Return the local bounds of the shape in x, y and z directions
    @Override
    public void getLocalBounds(Vector3 min, Vector3 max) {

        // Maximum bounds
        max.setX(radius + margin);
        max.setY(halfHeight + margin);
        max.setZ(max.getX());

        // Minimum bounds
        min.setX(-max.getX());
        min.setY(-max.getY());
        min.setZ(min.getX());
    }

    @Override
    public CollisionShape clone() {
        return new CylinderShape(this);
    }

}
