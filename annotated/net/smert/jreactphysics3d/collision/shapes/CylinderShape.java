/*
 * ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/
 * Copyright (c) 2010-2013 Daniel Chappuis
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the
 * use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim
 *    that you wrote the original software. If you use this software in a
 *    product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * This file has been modified during the port to Java and differ from the source versions.
 */
package net.smert.jreactphysics3d.collision.shapes;

import units.qual.Dimensionless;
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
@Dimensionless
public class CylinderShape extends CollisionShape {

    // Half height of the cylinder
    private final @Dimensionless float halfHeight;

    // Radius of the base
    private final @Dimensionless float radius;

    // Constructor
    public CylinderShape(@Dimensionless float radius, @Dimensionless float height, @Dimensionless float margin) {
        super(CollisionShapeType.CYLINDER, margin);
        assert (height > ((@Dimensionless float) (0.0f)));
        assert (radius > ((@Dimensionless float) (0.0f)));
        halfHeight = height * ((@Dimensionless float) (0.5f));
        this.radius = radius;
    }

    // Copy-constructor
    public CylinderShape(@Dimensionless CylinderShape shape) {
        super(shape);
        halfHeight = shape.halfHeight;
        radius = shape.radius;
    }

    // Return the radius
    public @Dimensionless float getRadius(@Dimensionless CylinderShape this) {
        return radius;
    }

    // Return the height
    public @Dimensionless float getHeight(@Dimensionless CylinderShape this) {
        return halfHeight + halfHeight;
    }

    // Test equality between two cylinder shapes
    @Override
    public @Dimensionless boolean isEqualTo(@Dimensionless CylinderShape this, @Dimensionless CollisionShape otherCollisionShape) {
        @Dimensionless
        CylinderShape otherShape = (@Dimensionless CylinderShape) otherCollisionShape;
        return (radius == otherShape.radius && halfHeight == otherShape.halfHeight);
    }

    // Return a local support point in a given direction with the object margin
    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithMargin(@Dimensionless CylinderShape this, @Dimensionless Vector3 direction, @Dimensionless Vector3 supportPoint) {

        // Compute the support point without the margin
        getLocalSupportPointWithoutMargin(direction, supportPoint);

        // Add the margin to the support point
        @Dimensionless
        Vector3 unitDirection = new @Dimensionless Vector3(((@Dimensionless float) (0.0f)), ((@Dimensionless float) (1.0f)), ((@Dimensionless float) (0.0f)));
        if (direction.lengthSquare() >= Defaults.MACHINE_EPSILON * Defaults.MACHINE_EPSILON) {
            unitDirection.set(direction).normalize();
        }

        return supportPoint.add(unitDirection.multiply(margin));
    }

    // Return a local support point in a given direction without the object margin
    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithoutMargin(@Dimensionless CylinderShape this, @Dimensionless Vector3 direction, @Dimensionless Vector3 supportPoint) {

        @Dimensionless
        float uDotv = direction.getY();
        @Dimensionless
        Vector3 w = new @Dimensionless Vector3(direction.getX(), ((@Dimensionless float) (0.0f)), direction.getZ());
        @Dimensionless
        float lengthW = Mathematics.Sqrt(direction.getX() * direction.getX() + direction.getZ() * direction.getZ());

        if (lengthW > Defaults.MACHINE_EPSILON) {
            if (uDotv < ((@Dimensionless float) (0.0f))) {
                supportPoint.setY(-halfHeight);
            } else {
                supportPoint.setY(halfHeight);
            }
            supportPoint.add(w.multiply(radius / lengthW));
        } else {
            if (uDotv < ((@Dimensionless float) (0.0f))) {
                supportPoint.setY(-halfHeight);
            } else {
                supportPoint.setY(halfHeight);
            }
        }

        return supportPoint;
    }

    // Return the local inertia tensor of the cylinder
    @Override
    public void computeLocalInertiaTensor(@Dimensionless CylinderShape this, @Dimensionless Matrix3x3 tensor, @Dimensionless float mass) {
        @Dimensionless
        float height = ((@Dimensionless float) (2.0f)) * halfHeight;
        @Dimensionless
        float diag = (((@Dimensionless float) (1.0f)) / ((@Dimensionless float) (12.0f))) * mass * (((@Dimensionless int) (3)) * radius * radius + height * height);
        tensor.set(diag, ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)),
                ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.5f)) * mass * radius * radius, ((@Dimensionless float) (0.0f)),
                ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)), diag);
    }

    // Return the local bounds of the shape in x, y and z directions
    @Override
    public void getLocalBounds(@Dimensionless CylinderShape this, @Dimensionless Vector3 min, @Dimensionless Vector3 max) {

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
    public @Dimensionless CollisionShape clone(@Dimensionless CylinderShape this) {
        return new @Dimensionless CylinderShape(this);
    }

}
