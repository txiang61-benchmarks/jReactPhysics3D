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
package net.smert.jreactphysics3d.mathematics;

import units.qual.Dimensionless;
import java.util.Objects;

/**
 * This class represents a position and an orientation in 3D. It can also be seen as representing a translation and a
 * rotation.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Transform {

    // Orientation
    private final @Dimensionless Quaternion orientation;

    // Position
    private final @Dimensionless Vector3 position;

    // Constructor
    public Transform() {
        orientation = new @Dimensionless Quaternion().identity();
        position = new @Dimensionless Vector3();
    }

    // Constructor with arguments
    public Transform(@Dimensionless Vector3 position, @Dimensionless Matrix3x3 orientation) {
        this.orientation = new @Dimensionless Quaternion(orientation);
        this.position = new @Dimensionless Vector3(position);
    }

    // Constructor with arguments
    public Transform(@Dimensionless Vector3 position, @Dimensionless Quaternion orientation) {
        this.orientation = new @Dimensionless Quaternion(orientation);
        this.position = new @Dimensionless Vector3(position);
    }

    // Copy-constructor
    public Transform(Transform transform) {
        orientation = new @Dimensionless Quaternion(transform.orientation);
        position = new @Dimensionless Vector3(transform.position);
    }

    // Return the rotation matrix
    public Quaternion getOrientation(@Dimensionless Transform this) {
        return orientation;
    }

    // Return the position of the transform
    public Vector3 getPosition(@Dimensionless Transform this) {
        return position;
    }

    // Set the transform from an OpenGL transform matrix
    public @Dimensionless Transform fromOpenGL(@Dimensionless Transform this, @Dimensionless float @Dimensionless [] openglMatrix) {
        @Dimensionless
        Matrix3x3 matrix = new @Dimensionless Matrix3x3(
                openglMatrix[((@Dimensionless int) (0))], openglMatrix[((@Dimensionless int) (4))], openglMatrix[((@Dimensionless int) (8))],
                openglMatrix[((@Dimensionless int) (1))], openglMatrix[((@Dimensionless int) (5))], openglMatrix[((@Dimensionless int) (9))],
                openglMatrix[((@Dimensionless int) (2))], openglMatrix[((@Dimensionless int) (6))], openglMatrix[((@Dimensionless int) (10))]);
        orientation.fromMatrix(matrix);
        position.set(openglMatrix[((@Dimensionless int) (12))], openglMatrix[((@Dimensionless int) (13))], openglMatrix[((@Dimensionless int) (14))]);
        return this;
    }

    // Set the transform to the identity transform
    public @Dimensionless Transform identity(@Dimensionless Transform this) {
        position.zero();
        orientation.identity();
        return this;
    }

    // Return the inverse of the transform
    public @Dimensionless Transform inverse(@Dimensionless Transform this) {
        orientation.inverse();
        position.invert();
        @Dimensionless
        Matrix3x3 invMatrix = orientation.getMatrix(new @Dimensionless Matrix3x3());
        @Dimensionless
        Vector3 invPosition = new @Dimensionless Vector3();
        invMatrix.multiply(position, invPosition);
        position.set(invPosition);
        return this;
    }

    // Operator of multiplication of a transform with another one
    public @Dimensionless Transform multiply(@Dimensionless Transform this, @Dimensionless Transform transform) {
        @Dimensionless
        Matrix3x3 matrix = orientation.getMatrix(new @Dimensionless Matrix3x3());
        orientation.multiply(transform.orientation);
        @Dimensionless
        Vector3 newPosition = new @Dimensionless Vector3();
        matrix.multiply(transform.position, newPosition);
        position.add(newPosition);
        return this;
    }

    // Assignment operator
    public Transform set(@Dimensionless Transform this, Transform transform) {
        orientation.set(transform.orientation);
        position.set(transform.position);
        return this;
    }

    // Set the rotation matrix of the transform
    public @Dimensionless Transform setOrientation(@Dimensionless Transform this, @Dimensionless Quaternion orientation) {
        this.orientation.set(orientation);
        return this;
    }

    // Set the origin of the transform
    public @Dimensionless Transform setPosition(@Dimensionless Transform this, @Dimensionless Vector3 position) {
        this.position.set(position);
        return this;
    }

    // Get the OpenGL matrix of the transform
    public @Dimensionless float @Dimensionless [] getOpenGLMatrix(@Dimensionless Transform this, @Dimensionless float @Dimensionless [] openglMatrix) {
        @Dimensionless
        Matrix3x3 matrix = new @Dimensionless Matrix3x3();
        orientation.getMatrix(matrix);
        openglMatrix[((@Dimensionless int) (0))] = matrix.m00;
        openglMatrix[((@Dimensionless int) (1))] = matrix.m10;
        openglMatrix[((@Dimensionless int) (2))] = matrix.m20;
        openglMatrix[((@Dimensionless int) (3))] = ((@Dimensionless float) (0.0f));
        openglMatrix[((@Dimensionless int) (4))] = matrix.m01;
        openglMatrix[((@Dimensionless int) (5))] = matrix.m11;
        openglMatrix[((@Dimensionless int) (6))] = matrix.m21;
        openglMatrix[((@Dimensionless int) (7))] = ((@Dimensionless float) (0.0f));
        openglMatrix[((@Dimensionless int) (8))] = matrix.m02;
        openglMatrix[((@Dimensionless int) (9))] = matrix.m12;
        openglMatrix[((@Dimensionless int) (10))] = matrix.m22;
        openglMatrix[((@Dimensionless int) (11))] = ((@Dimensionless float) (0.0f));
        openglMatrix[((@Dimensionless int) (12))] = position.x;
        openglMatrix[((@Dimensionless int) (13))] = position.y;
        openglMatrix[((@Dimensionless int) (14))] = position.z;
        openglMatrix[((@Dimensionless int) (15))] = ((@Dimensionless float) (1.0f));
        return openglMatrix;
    }

    // Return the transformed vector
    public @Dimensionless Vector3 multiply(@Dimensionless Transform this, @Dimensionless Vector3 vector, @Dimensionless Vector3 vectorOut) {
        @Dimensionless
        Matrix3x3 matrix = orientation.getMatrix(new @Dimensionless Matrix3x3());
        return matrix.multiply(vector, vectorOut).add(position);
    }

    // Return an interpolated transform
    public static Transform Interpolate(Transform oldTransform, Transform newTransform, float interpolationFactor, Transform outTransform) {
        assert (interpolationFactor >= ((@Dimensionless float) (0.0f)) && interpolationFactor <= ((@Dimensionless float) (1.0f)));
        @Dimensionless
        Quaternion interOrientation = new @Dimensionless Quaternion();
        @Dimensionless
        Vector3 interPosition = new @Dimensionless Vector3();
        Quaternion.Slerp(oldTransform.orientation, newTransform.orientation, interpolationFactor, interOrientation);
        Vector3.Lerp(oldTransform.position, newTransform.position, interpolationFactor, interPosition);
        outTransform.setOrientation(interOrientation);
        outTransform.setPosition(interPosition);
        return outTransform;
    }

    @Override
    public @Dimensionless int hashCode(@Dimensionless Transform this) {
        @Dimensionless
        int hash = ((@Dimensionless int) (3));
        hash = ((@Dimensionless int) (71)) * hash + Objects.hashCode(this.position);
        hash = ((@Dimensionless int) (71)) * hash + Objects.hashCode(this.orientation);
        return hash;
    }

    @Override
    public boolean equals(@Dimensionless Transform this, Object obj) {

        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final @Dimensionless Transform other = (@Dimensionless Transform) obj;
        if (!Objects.equals(this.position, other.position)) {
            return false;
        }
        return Objects.equals(this.orientation, other.orientation);
    }

    @Override
    public @Dimensionless String toString(@Dimensionless Transform this) {
        return "(position= " + position + ", orientation= " + orientation + ")";
    }

}
