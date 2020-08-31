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
import java.util.Objects;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This abstract class represents the collision shape associated with a body that is used during the narrow-phase
 * collision detection.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public abstract class CollisionShape {

    // Margin used for the GJK collision detection algorithm
    protected @Dimensionless float margin;

    // Current number of similar created shapes
    protected @Dimensionless int numSimilarCreatedShapes;

    // Type of the collision shape
    protected final @Dimensionless CollisionShapeType type;

    // Constructor
    public CollisionShape(@Dimensionless CollisionShapeType type, @Dimensionless float margin) {
        assert (margin >= ((@Dimensionless float) (0.0f)));
        this.margin = margin;
        numSimilarCreatedShapes = ((@Dimensionless int) (0));
        this.type = type;
    }

    // Copy-constructor
    public CollisionShape(@Dimensionless CollisionShape shape) {
        margin = shape.margin;
        numSimilarCreatedShapes = shape.numSimilarCreatedShapes;
        type = shape.type;
    }

    // Return the current object margin
    public @Dimensionless float getMargin(@Dimensionless CollisionShape this) {
        return margin;
    }

    // Return the number of similar created shapes
    public @Dimensionless int getNumSimilarCreatedShapes(@Dimensionless CollisionShape this) {
        return numSimilarCreatedShapes;
    }

    // Decrement the number of similar allocated collision shapes
    public void decrementNumSimilarCreatedShapes(@Dimensionless CollisionShape this) {
        numSimilarCreatedShapes--;
    }

    // Increment the number of similar allocated collision shapes
    public void incrementNumSimilarCreatedShapes(@Dimensionless CollisionShape this) {
        numSimilarCreatedShapes++;
    }

    // Return the type of the collision shape
    public @Dimensionless CollisionShapeType getType(@Dimensionless CollisionShape this) {
        return type;
    }

    // Update the AABB of a body using its collision shape
    public void updateAABB(@Dimensionless CollisionShape this, AABB aabb, Transform transform) {

        // Get the local bounds in x,y and z direction
        @Dimensionless
        Vector3 minBounds = new @Dimensionless Vector3();
        @Dimensionless
        Vector3 maxBounds = new @Dimensionless Vector3();
        getLocalBounds(minBounds, maxBounds);

        // Rotate the local bounds according to the orientation of the body
        @Dimensionless
        Matrix3x3 worldAxis = transform.getOrientation().getMatrix(new @Dimensionless Matrix3x3());
        worldAxis.abs();
        @Dimensionless
        Vector3 worldMinBounds = new @Dimensionless Vector3(worldAxis.getColumn(((@Dimensionless int) (0))).dot(minBounds),
                worldAxis.getColumn(((@Dimensionless int) (1))).dot(minBounds),
                worldAxis.getColumn(((@Dimensionless int) (2))).dot(minBounds));
        @Dimensionless
        Vector3 worldMaxBounds = new @Dimensionless Vector3(worldAxis.getColumn(((@Dimensionless int) (0))).dot(maxBounds),
                worldAxis.getColumn(((@Dimensionless int) (1))).dot(maxBounds),
                worldAxis.getColumn(((@Dimensionless int) (2))).dot(maxBounds));

        // Compute the minimum and maximum coordinates of the rotated extents
        @Dimensionless
        Vector3 minCoordinates = new @Dimensionless Vector3(transform.getPosition()).add(worldMinBounds);
        @Dimensionless
        Vector3 maxCoordinates = new @Dimensionless Vector3(transform.getPosition()).add(worldMaxBounds);

        // Update the AABB with the new minimum and maximum coordinates
        aabb.setMin(minCoordinates);
        aabb.setMax(maxCoordinates);
    }

    // Test equality between two collision shapes of the same type (same derived classes).
    public abstract @Dimensionless boolean isEqualTo(@Dimensionless CollisionShape this, @Dimensionless CollisionShape otherCollisionShape);

    // Return a local support point in a given direction with the object margin
    public abstract @Dimensionless Vector3 getLocalSupportPointWithMargin(@Dimensionless CollisionShape this, @Dimensionless Vector3 direction, @Dimensionless Vector3 supportPoint);

    // Return a local support point in a given direction without the object margin
    public abstract @Dimensionless Vector3 getLocalSupportPointWithoutMargin(@Dimensionless CollisionShape this, @Dimensionless Vector3 direction, @Dimensionless Vector3 supportPoint);

    // Return the local inertia tensor of the collision shapes
    public abstract void computeLocalInertiaTensor(@Dimensionless CollisionShape this, @Dimensionless Matrix3x3 tensor, @Dimensionless float mass);

    // Return the local bounds of the shape in x, y and z directions
    public abstract void getLocalBounds(@Dimensionless CollisionShape this, @Dimensionless Vector3 min, @Dimensionless Vector3 max);

    @Override
    public abstract @Dimensionless CollisionShape clone(@Dimensionless CollisionShape this);

    @Override
    public @Dimensionless int hashCode(@Dimensionless CollisionShape this) {
        @Dimensionless
        int hash = ((@Dimensionless int) (7));
        hash = ((@Dimensionless int) (67)) * hash + Float.floatToIntBits(margin);
        hash = ((@Dimensionless int) (67)) * hash + numSimilarCreatedShapes;
        hash = ((@Dimensionless int) (67)) * hash + Objects.hashCode(type);
        return hash;
    }

    @Override
    public @Dimensionless boolean equals(@Dimensionless CollisionShape this, @Dimensionless Object obj) {

        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final @Dimensionless CollisionShape other = (@Dimensionless CollisionShape) obj;
        if (type == other.type) {
            return false;
        }
        if (Float.floatToIntBits(margin) != Float.floatToIntBits(other.margin)) {
            return false;
        }
        return numSimilarCreatedShapes != other.numSimilarCreatedShapes;
    }

    @Override
    public @Dimensionless String toString(@Dimensionless CollisionShape this) {
        return "{type=" + type + " margin=" + margin + "}";
    }

}
