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
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a 3D box shape. Those axis are unit length. The three extents are half-widths of the box along
 * the three axis x, y, z local axis. The "transform" of the corresponding rigid body will give an orientation and a
 * position to the box. This collision shape uses an extra margin distance around it for collision detection purpose.
 * The default margin is 4cm (if your units are meters, which is recommended). In case, you want to simulate small
 * objects (smaller than the margin distance), you might want to reduce the margin by specifying your own margin
 * distance using the "margin" parameter in the constructor of the box shape. Otherwise, it is recommended to use the
 * default margin distance by not using the "margin" parameter in the constructor.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
@Dimensionless
public class BoxShape extends CollisionShape {

    // Extent sizes of the box in the x, y and z direction
    private final @Dimensionless Vector3 extent;

    // Constructor
    public BoxShape(@Dimensionless Vector3 extent, @Dimensionless float margin) {
        super(CollisionShapeType.BOX, margin);
        assert (extent.getX() > ((@Dimensionless float) (0.0f)) && extent.getX() > margin);
        assert (extent.getY() > ((@Dimensionless float) (0.0f)) && extent.getY() > margin);
        assert (extent.getZ() > ((@Dimensionless float) (0.0f)) && extent.getZ() > margin);
        this.extent = new @Dimensionless Vector3(extent);
    }

    // Copy-constructor
    public BoxShape(@Dimensionless BoxShape shape) {
        super(shape);
        extent = new @Dimensionless Vector3(shape.extent);
    }

    // Return the extents of the box
    public @Dimensionless Vector3 getExtent(@Dimensionless BoxShape this) {
        return extent;
    }

    // Test equality between two box shapes
    @Override
    public @Dimensionless boolean isEqualTo(@Dimensionless BoxShape this, @Dimensionless CollisionShape otherCollisionShape) {
        @Dimensionless
        BoxShape otherShape = (@Dimensionless BoxShape) otherCollisionShape;
        return extent.equals(otherShape.extent);
    }

    // Return a local support point in a given direction with the object margin
    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithMargin(@Dimensionless BoxShape this, @Dimensionless Vector3 direction, @Dimensionless Vector3 supportPoint) {
        return supportPoint.set(
                direction.getX() < ((@Dimensionless float) (0.0f)) ? -extent.getX() - margin : extent.getX() + margin,
                direction.getY() < ((@Dimensionless float) (0.0f)) ? -extent.getY() - margin : extent.getY() + margin,
                direction.getZ() < ((@Dimensionless float) (0.0f)) ? -extent.getZ() - margin : extent.getZ() + margin);
    }

    // Return a local support point in a given direction without the objec margin
    @Override
    public @Dimensionless Vector3 getLocalSupportPointWithoutMargin(@Dimensionless BoxShape this, @Dimensionless Vector3 direction, @Dimensionless Vector3 supportPoint) {
        return supportPoint.set(
                direction.getX() < ((@Dimensionless float) (0.0f)) ? -extent.getX() : extent.getX(),
                direction.getY() < ((@Dimensionless float) (0.0f)) ? -extent.getY() : extent.getY(),
                direction.getZ() < ((@Dimensionless float) (0.0f)) ? -extent.getZ() : extent.getZ());
    }

    // Return the local inertia tensor of the collision shape
    @Override
    public void computeLocalInertiaTensor(@Dimensionless BoxShape this, @Dimensionless Matrix3x3 tensor, @Dimensionless float mass) {
        @Dimensionless
        float factor = (((@Dimensionless float) (1.0f)) / ((@Dimensionless float) (3.0f))) * mass;
        @Dimensionless
        float xSquare = extent.getX() * extent.getX();
        @Dimensionless
        float ySquare = extent.getY() * extent.getY();
        @Dimensionless
        float zSquare = extent.getZ() * extent.getZ();
        tensor.set(factor * (ySquare + zSquare), ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)),
                ((@Dimensionless float) (0.0f)), factor * (xSquare + zSquare), ((@Dimensionless float) (0.0f)),
                ((@Dimensionless float) (0.0f)), ((@Dimensionless float) (0.0f)), factor * (xSquare + ySquare));
    }

    // Return the local bounds of the shape in x, y and z directions
    // This method is used to compute the AABB of the box
    @Override
    public void getLocalBounds(@Dimensionless BoxShape this, @Dimensionless Vector3 min, @Dimensionless Vector3 max) {

        // Maximum bounds
        max.set(extent.getX(), extent.getY(), extent.getZ());
        max.add(new @Dimensionless Vector3(margin, margin, margin));

        // Minimum bounds
        min.set(-max.getX(), -max.getY(), -max.getZ());
    }

    @Override
    public @Dimensionless CollisionShape clone(@Dimensionless BoxShape this) {
        return new @Dimensionless BoxShape(this);
    }

}
