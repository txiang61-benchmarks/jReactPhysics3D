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
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a sphere collision shape that is centered at the origin and defined by its radius. This
 * collision shape does not have an explicit object margin distance. The margin is implicitly the radius of the sphere.
 * Therefore, no need to specify an object margin for a sphere shape.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
@Dimensionless
public class SphereShape extends CollisionShape {

    // Radius of the sphere
    private final @Dimensionless float radius;

    // Constructor
    public SphereShape(@Dimensionless float radius, @Dimensionless float margin) {
        super(CollisionShapeType.SPHERE, radius + margin);
        assert (radius > ((@Dimensionless float) (0.0f)));
        this.radius = radius;
    }

    // Copy-constructor
    public SphereShape(@Dimensionless SphereShape shape) {
        super(shape);
        radius = shape.radius;
    }

    // Get the radius of the sphere
    public @Dimensionless float getRadius(@Dimensionless SphereShape this) {
        return radius;
    }

    // Update the AABB of a body using its collision shape
    @Override
    public void updateAABB(@Dimensionless SphereShape this, @Dimensionless AABB aabb, @Dimensionless Transform transform) {

        // Get the local extents in x,y and z direction
        @Dimensionless
        Vector3 extents = new @Dimensionless Vector3(radius, radius, radius);

        // Update the AABB with the new minimum and maximum coordinates
        aabb.setMin(new @Dimensionless Vector3(transform.getPosition()).subtract(extents));
        aabb.setMax(new @Dimensionless Vector3(transform.getPosition()).add(extents));
    }

    // Test equality between two sphere shapes
    @Override
    public @Dimensionless boolean isEqualTo(@Dimensionless SphereShape this, @Dimensionless CollisionShape otherCollisionShape) {
        @Dimensionless
        SphereShape otherShape = (@Dimensionless SphereShape) otherCollisionShape;
        return (radius == otherShape.radius);
    }

    // Return a local support point in a given direction with the object margin
    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithMargin(@Dimensionless SphereShape this, @Dimensionless Vector3 direction, @Dimensionless Vector3 supportPoint) {

        // If the direction vector is not the zero vector
        if (direction.lengthSquare() >= Defaults.MACHINE_EPSILON * Defaults.MACHINE_EPSILON) {

            // Return the support point of the sphere in the given direction
            return supportPoint.set(direction).normalize().multiply(getMargin());
        }

        // If the direction vector is the zero vector we return a point on the
        // boundary of the sphere
        return supportPoint.set(((@Dimensionless float) (0.0f)), getMargin(), ((@Dimensionless float) (0.0f)));
    }

    // Return a local support point in a given direction without the object margin
    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithoutMargin(@Dimensionless SphereShape this, @Dimensionless Vector3 direction, @Dimensionless Vector3 supportPoint) {

        // Return the center of the sphere (the radius is taken into account in the object margin)
        return supportPoint.zero();
    }

    // Return the local inertia tensor of the sphere
    @Override
    public void computeLocalInertiaTensor(@Dimensionless SphereShape this, @Dimensionless Matrix3x3 tensor, @Dimensionless float mass) {
        @Dimensionless
        float diag = ((@Dimensionless float) (0.4f)) * mass * radius * radius;
        tensor.set(diag, ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)),
                ((@Dimensionless float) (0.0f)), diag, ((@Dimensionless float) (0.0f)),
                ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)), diag);
    }

    // Return the local bounds of the shape in x, y and z directions.
    // This method is used to compute the AABB of the box
    @Override
    public void getLocalBounds(@Dimensionless SphereShape this, @Dimensionless Vector3 min, @Dimensionless Vector3 max) {

        // Maximum bounds
        max.setX(radius + margin);
        max.setY(max.getX());
        max.setZ(max.getX());

        // Minimum bounds
        min.setX(-radius - margin);
        min.setY(min.getX());
        min.setZ(min.getX());
    }

    @Override
    public @Dimensionless CollisionShape clone(@Dimensionless SphereShape this) {
        return new @Dimensionless SphereShape(this);
    }

}
