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
 * This class represents a cone collision shape centered at the origin and aligned with the Y axis. The cone is defined
 * by its height and by the radius of its base. The center of the cone is at the half of the height. The "transform" of
 * the corresponding rigid body gives an orientation and a position to the cone. This collision shape uses an extra
 * margin distance around it for collision detection purpose. The default margin is 4cm (if your units are meters, which
 * is recommended). In case, you want to simulate small objects (smaller than the margin distance), you might want to
 * reduce the margin by specifying your own margin distance using the "margin" parameter in the constructor of the cone
 * shape. Otherwise, it is recommended to use the default margin distance by not using the "margin" parameter in the
 * constructor.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
@Dimensionless
public class ConeShape extends CollisionShape {

    // Half height of the cone
    private final @Dimensionless float halfHeight;

    // Radius of the base
    private final @Dimensionless float radius;

    // sine of the semi angle at the apex point
    private final @Dimensionless float sinTheta;

    // Constructor
    public ConeShape(@Dimensionless float radius, @Dimensionless float height, @Dimensionless float margin) {
        super(CollisionShapeType.CONE, margin);
        assert (height > ((@Dimensionless float) (0.0f)));
        assert (radius > ((@Dimensionless float) (0.0f)));
        halfHeight = height * ((@Dimensionless float) (0.5f));
        this.radius = radius;

        // Compute the sine of the semi-angle at the apex point
        sinTheta = radius / (Mathematics.Sqrt(radius * radius + height * height));
    }

    // Copy-constructor
    public ConeShape(@Dimensionless ConeShape shape) {
        super(shape);
        halfHeight = shape.halfHeight;
        radius = shape.radius;
        sinTheta = shape.sinTheta;
    }

    // Return the radius
    public @Dimensionless float getRadius(@Dimensionless ConeShape this) {
        return radius;
    }

    // Return the height
    public @Dimensionless float getHeight(@Dimensionless ConeShape this) {
        return halfHeight + halfHeight;
    }

    // Test equality between two cone shapes
    @Override
    public @Dimensionless boolean isEqualTo(@Dimensionless ConeShape this, @Dimensionless CollisionShape otherCollisionShape) {
        @Dimensionless
        ConeShape otherShape = (@Dimensionless ConeShape) otherCollisionShape;
        return (radius == otherShape.radius && halfHeight == otherShape.halfHeight);
    }

    // Return a local support point in a given direction with the object margin
    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithMargin(@Dimensionless ConeShape this, @Dimensionless Vector3 direction, @Dimensionless Vector3 supportPoint) {

        // Compute the support point without the margin
        getLocalSupportPointWithoutMargin(direction, supportPoint);

        // Add the margin to the support point
        @Dimensionless
        Vector3 unitDirection = new @Dimensionless Vector3(((@Dimensionless float) (0.0f)), - ((@Dimensionless float) (1.0f)), ((@Dimensionless float) (0.0f)));
        if (direction.lengthSquare() >= Defaults.MACHINE_EPSILON * Defaults.MACHINE_EPSILON) {
            unitDirection.set(direction).normalize();
        }

        return supportPoint.add(unitDirection.multiply(margin));
    }

    // Return a local support point in a given direction without the object margin
    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithoutMargin(@Dimensionless ConeShape this, @Dimensionless Vector3 direction, @Dimensionless Vector3 supportPoint) {

        @Dimensionless
        Vector3 v = direction;
        @Dimensionless
        float sinThetaTimesLengthV = sinTheta * v.length();

        if (v.getY() > sinThetaTimesLengthV) {
            supportPoint.set(((@Dimensionless float) (0.0f)), halfHeight, ((@Dimensionless float) (0.0f)));
        } else {
            @Dimensionless
            float projectedLength = Mathematics.Sqrt(v.getX() * v.getX() + v.getZ() * v.getZ());
            if (projectedLength > Defaults.MACHINE_EPSILON) {
                @Dimensionless
                float d = radius / projectedLength;
                supportPoint.set(v.getX() * d, -halfHeight, v.getZ() * d);
            } else {
                supportPoint.set(((@Dimensionless float) (0.0f)), -halfHeight, ((@Dimensionless float) (0.0f)));
            }
        }

        return supportPoint;
    }

    // Return the local inertia tensor of the collision shape
    @Override
    public void computeLocalInertiaTensor(@Dimensionless ConeShape this, @Dimensionless Matrix3x3 tensor, @Dimensionless float mass) {
        @Dimensionless
        float rSquare = radius * radius;
        @Dimensionless
        float diagXZ = ((@Dimensionless float) (0.15f)) * mass * (rSquare + halfHeight);
        tensor.set(diagXZ, ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)),
                ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.3f)) * mass * rSquare,
                ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)), diagXZ);
    }

    // Return the local bounds of the shape in x, y and z directions
    @Override
    public void getLocalBounds(@Dimensionless ConeShape this, @Dimensionless Vector3 min, @Dimensionless Vector3 max) {

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
    public @Dimensionless CollisionShape clone(@Dimensionless ConeShape this) {
        return new @Dimensionless ConeShape(this);
    }

}
