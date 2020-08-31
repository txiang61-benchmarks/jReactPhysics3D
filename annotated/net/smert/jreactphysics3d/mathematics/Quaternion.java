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
import net.smert.jreactphysics3d.configuration.Defaults;

/**
 * This class represents a quaternion. We use the notation : q = (x*i, y*j, z*k, w) to represent a quaternion.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Quaternion {

    // Component x
    @Dimensionless
    float x;

    // Component y
    @Dimensionless
    float y;

    // Component z
    @Dimensionless
    float z;

    // Component w
    @Dimensionless
    float w;

    // Constructor
    public Quaternion() {
        zero();
    }

    // Constructor with arguments
    public Quaternion(@Dimensionless float x, @Dimensionless float y, @Dimensionless float z, @Dimensionless float w) {
        set(x, y, z, w);
    }

    // Create a unit quaternion from a rotation matrix
    public Quaternion(Matrix3x3 matrix) {
        fromMatrix(matrix);
    }

    // Copy-constructor
    public Quaternion(Quaternion quaternion) {
        set(quaternion);
    }

    // Constructor with the component w and the vector v=(x y z)
    public Quaternion(@Dimensionless Vector3 vector, @Dimensionless float w) {
        set(vector, w);
    }

    // Scalar product between two quaternions
    public @Dimensionless float dot(@Dimensionless Quaternion this, @Dimensionless Quaternion quaternion) {
        return x * quaternion.x + y * quaternion.y + z * quaternion.z + w * quaternion.w;
    }

    public @Dimensionless float getW(@Dimensionless Quaternion this) {
        return w;
    }

    public @Dimensionless float getX(@Dimensionless Quaternion this) {
        return x;
    }

    public @Dimensionless float getY(@Dimensionless Quaternion this) {
        return y;
    }

    public @Dimensionless float getZ(@Dimensionless Quaternion this) {
        return z;
    }

    // Return the length of the quaternion (public )
    public @Dimensionless float length(@Dimensionless Quaternion this) {
        return Mathematics.Sqrt(x * x + y * y + z * z + w * w);
    }

    // Return the square of the length of the quaternion
    public @Dimensionless float lengthSquare(@Dimensionless Quaternion this) {
        return x * x + y * y + z * z + w * w;
    }

    // Overloaded operator for addition with assignment
    public @Dimensionless Quaternion add(@Dimensionless Quaternion this, @Dimensionless Quaternion quaternion) {
        x += quaternion.x;
        y += quaternion.y;
        z += quaternion.z;
        w += quaternion.w;
        return this;
    }

    // Return the conjugate of the quaternion (public )
    public @Dimensionless Quaternion conjugate(@Dimensionless Quaternion this) {
        x = -x;
        y = -y;
        z = -z;
        return this;
    }

    public final Quaternion fromMatrix(@Dimensionless Quaternion this, Matrix3x3 matrix) {

        // Get the trace of the matrix
        float r, s, trace = matrix.getTrace();

        if (trace < ((@Dimensionless float) (0.0f))) {
            if (matrix.m11 > matrix.m00) {
                if (matrix.m22 > matrix.m11) {
                    r = Mathematics.Sqrt(matrix.m22 - matrix.m00 - matrix.m11 + ((@Dimensionless float) (1.0f)));
                    s = ((@Dimensionless float) (0.5f)) / r;

                    // Compute the quaternion
                    x = (matrix.m20 + matrix.m02) * s;
                    y = (matrix.m12 + matrix.m21) * s;
                    z = ((@Dimensionless float) (0.5f)) * r;
                    w = (matrix.m10 - matrix.m01) * s;
                } else {
                    r = Mathematics.Sqrt(matrix.m11 - matrix.m22 - matrix.m00 + ((@Dimensionless float) (1.0f)));
                    s = ((@Dimensionless float) (0.5f)) / r;

                    // Compute the quaternion
                    x = (matrix.m01 + matrix.m10) * s;
                    y = ((@Dimensionless float) (0.5f)) * r;
                    z = (matrix.m12 + matrix.m21) * s;
                    w = (matrix.m02 - matrix.m20) * s;
                }
            } else if (matrix.m22 > matrix.m00) {
                r = Mathematics.Sqrt(matrix.m22 - matrix.m00 - matrix.m11 + ((@Dimensionless float) (1.0f)));
                s = ((@Dimensionless float) (0.5f)) / r;

                // Compute the quaternion
                x = (matrix.m20 + matrix.m02) * s;
                y = (matrix.m12 + matrix.m21) * s;
                z = ((@Dimensionless float) (0.5f)) * r;
                w = (matrix.m10 - matrix.m01) * s;
            } else {
                r = Mathematics.Sqrt(matrix.m00 - matrix.m11 - matrix.m22 + ((@Dimensionless float) (1.0f)));
                s = ((@Dimensionless float) (0.5f)) / r;

                // Compute the quaternion
                x = ((@Dimensionless float) (0.5f)) * r;
                y = (matrix.m01 + matrix.m10) * s;
                z = (matrix.m20 - matrix.m02) * s;
                w = (matrix.m21 - matrix.m12) * s;
            }
        } else {
            r = Mathematics.Sqrt(trace + ((@Dimensionless float) (1.0f)));
            s = ((@Dimensionless float) (0.5f)) / r;

            // Compute the quaternion
            x = (matrix.m21 - matrix.m12) * s;
            y = (matrix.m02 - matrix.m20) * s;
            z = (matrix.m10 - matrix.m01) * s;
            w = ((@Dimensionless float) (0.5f)) * r;
        }

        return this;
    }

    // Set to the identity quaternion
    public Quaternion identity(@Dimensionless Quaternion this) {
        x = ((@Dimensionless float) (0.0f));
        y = ((@Dimensionless float) (0.0f));
        z = ((@Dimensionless float) (0.0f));
        w = ((@Dimensionless float) (1.0f));
        return this;
    }

    // Inverse the quaternion
    public Quaternion inverse(@Dimensionless Quaternion this) {

        // Get the square length of the quaternion
        @Dimensionless
        float lenSq = lengthSquare();
        assert (lenSq > Defaults.MACHINE_EPSILON);

        // Compute and return the inverse quaternion
        x /= -lenSq;
        y /= -lenSq;
        z /= -lenSq;
        w /= lenSq;
        return this;
    }

    // Overloaded operator for the multiplication with a constant
    public @Dimensionless Quaternion multiply(@Dimensionless Quaternion this, @Dimensionless float number) {
        x *= number;
        y *= number;
        z *= number;
        w *= number;
        return this;
    }

    // Overloaded operator for the multiplication of two quaternions
    public Quaternion multiply(@Dimensionless Quaternion this, Quaternion quaternion) {
        @Dimensionless
        Vector3 q1V = new @Dimensionless Vector3();
        getVectorV(q1V);
        @Dimensionless
        Vector3 q2V = new @Dimensionless Vector3();
        quaternion.getVectorV(q2V);
        @Dimensionless
        Vector3 newVector = new @Dimensionless Vector3(q2V).multiply(w)
                .add(new @Dimensionless Vector3(q1V).multiply(quaternion.w))
                .add(new @Dimensionless Vector3(q1V).cross(q2V));
        return set(
                newVector.getX(), newVector.getY(), newVector.getZ(),
                w * quaternion.w - q1V.dot(q2V));
    }

    // Normalize the quaternion
    public @Dimensionless Quaternion normalize(@Dimensionless Quaternion this) {
        @Dimensionless
        float len = length();
        assert (len > Defaults.MACHINE_EPSILON);
        @Dimensionless
        float lenInv = ((@Dimensionless float) (1.0f)) / len;
        x *= lenInv;
        y *= lenInv;
        z *= lenInv;
        w *= lenInv;
        return this;
    }

    // Set all the values
    public final @Dimensionless Quaternion set(@Dimensionless Quaternion this, @Dimensionless float x, @Dimensionless float y, @Dimensionless float z, @Dimensionless float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
        return this;
    }

    public final Quaternion set(@Dimensionless Quaternion this, Quaternion quaternion) {
        x = quaternion.x;
        y = quaternion.y;
        z = quaternion.z;
        w = quaternion.w;
        return this;
    }

    public final @Dimensionless Quaternion set(@Dimensionless Quaternion this, @Dimensionless Vector3 vector, @Dimensionless float w) {
        x = vector.x;
        y = vector.y;
        z = vector.z;
        this.w = w;
        return this;
    }

    public @Dimensionless Quaternion setW(@Dimensionless Quaternion this, @Dimensionless float w) {
        this.w = w;
        return this;
    }

    public @Dimensionless Quaternion setX(@Dimensionless Quaternion this, @Dimensionless float x) {
        this.x = x;
        return this;
    }

    public @Dimensionless Quaternion setY(@Dimensionless Quaternion this, @Dimensionless float y) {
        this.y = y;
        return this;
    }

    public @Dimensionless Quaternion setZ(@Dimensionless Quaternion this, @Dimensionless float z) {
        this.z = z;
        return this;
    }

    // Overloaded operator for substraction with assignment
    public @Dimensionless Quaternion subtract(@Dimensionless Quaternion this, @Dimensionless Quaternion quaternion) {
        x -= quaternion.x;
        y -= quaternion.y;
        z -= quaternion.z;
        w -= quaternion.w;
        return this;
    }

    // Set the quaternion to zero
    public final @Dimensionless Quaternion zero(@Dimensionless Quaternion this) {
        x = ((@Dimensionless float) (0.0f));
        y = ((@Dimensionless float) (0.0f));
        z = ((@Dimensionless float) (0.0f));
        w = ((@Dimensionless float) (0.0f));
        return this;
    }

    // Return the orientation matrix corresponding to this quaternion
    public Matrix3x3 getMatrix(@Dimensionless Quaternion this, Matrix3x3 matrix) {

        @Dimensionless
        float nQ = x * x + y * y + z * z + w * w;
        @Dimensionless
        float s = ((@Dimensionless float) (0.0f));

        if (nQ > ((@Dimensionless float) (0.0f))) {
            s = ((@Dimensionless float) (2.0f)) / nQ;
        }

        // Computations used for optimization (less multiplications)
        @Dimensionless
        float xs = x * s;
        @Dimensionless
        float ys = y * s;
        @Dimensionless
        float zs = z * s;
        @Dimensionless
        float wxs = w * xs;
        @Dimensionless
        float wys = w * ys;
        @Dimensionless
        float wzs = w * zs;
        @Dimensionless
        float xxs = x * xs;
        @Dimensionless
        float xys = x * ys;
        @Dimensionless
        float xzs = x * zs;
        @Dimensionless
        float yys = y * ys;
        @Dimensionless
        float yzs = y * zs;
        @Dimensionless
        float zzs = z * zs;

        // Create the matrix corresponding to the quaternion
        return matrix.set(((@Dimensionless float) (1.0f)) - yys - zzs, xys - wzs, xzs + wys,
                xys + wzs, ((@Dimensionless float) (1.0f)) - xxs - zzs, yzs - wxs,
                xzs - wys, yzs + wxs, ((@Dimensionless float) (1.0f)) - xxs - yys);
    }

    // Compute the rotation angle (in radians) and the rotation axis
    // This method is used to get the rotation angle (in radian) and the unit
    // rotation axis of an orientation quaternion.
    public @Dimensionless Vector3 getRotationAngleAxis(@Dimensionless Quaternion this, @Dimensionless Vector3 axis, @Dimensionless float @Dimensionless [] angle) {

        @Dimensionless
        Quaternion quaternion;

        // If the quaternion is unit
        if (length() == ((@Dimensionless double) (1.0))) {
            quaternion = this;
        } else {
            // We compute the unit quaternion
            quaternion = new @Dimensionless Quaternion(this).normalize();
        }

        // Compute the roation angle
        angle[((@Dimensionless int) (0))] = Mathematics.ArcCos(quaternion.w) * ((@Dimensionless float) (2.0f));

        // Compute the 3D rotation axis
        @Dimensionless
        Vector3 rotationAxis = new @Dimensionless Vector3(quaternion.x, quaternion.y, quaternion.z);

        // Normalize the rotation axis
        rotationAxis.normalize();

        // Set the rotation axis values
        return axis.set(rotationAxis);
    }

    // Return the vector v=(x y z) of the quaternion
    public @Dimensionless Vector3 getVectorV(@Dimensionless Quaternion this, @Dimensionless Vector3 vector) {
        return vector.set(x, y, z);
    }

    // Overloaded operator for the multiplication with a vector.
    // This methods rotates a point given the rotation of a quaternion.
    public @Dimensionless Vector3 multiply(@Dimensionless Quaternion this, @Dimensionless Vector3 vector, @Dimensionless Vector3 vectorOut) {
        @Dimensionless
        Quaternion c = new @Dimensionless Quaternion(this).conjugate();
        @Dimensionless
        Quaternion p = new @Dimensionless Quaternion(vector.x, vector.y, vector.z, ((@Dimensionless float) (0.0f)));
        new @Dimensionless Quaternion(this).multiply(p).multiply(c).getVectorV(vectorOut);
        return vectorOut;
    }

    // Compute the spherical linear interpolation between two quaternions.
    // The t argument has to be such that 0 <= t <= 1. This method is static.
    public static void Slerp(Quaternion oldQuaternion, Quaternion newQuaternion2, float t, Quaternion quaternionOut) {

        assert (t >= ((@Dimensionless float) (0.0f)) && t <= ((@Dimensionless float) (1.0f)));

        @Dimensionless
        float invert = ((@Dimensionless float) (1.0f));
        @Dimensionless
        Quaternion tempQ2 = new @Dimensionless Quaternion(newQuaternion2);

        // Compute cos(theta) using the quaternion scalar product
        @Dimensionless
        float cosineTheta = oldQuaternion.dot(newQuaternion2);

        // Take care of the sign of cosineTheta
        if (cosineTheta < ((@Dimensionless float) (0.0f))) {
            cosineTheta = -cosineTheta;
            invert = - ((@Dimensionless float) (1.0f));
        }

        // Because of precision, if cos(theta) is nearly 1,
        // therefore theta is nearly 0 and we can write
        // sin((1-t)*theta) as (1-t) and sin(t*theta) as t
        @Dimensionless
        float epsilon = ((@Dimensionless float) (0.00001f));
        if (((@Dimensionless int) (1)) - cosineTheta < epsilon) {
            quaternionOut.set(oldQuaternion).multiply(((@Dimensionless float) (1.0f)) - t).add(tempQ2.multiply(t * invert));
            return;
        }

        // Compute the theta angle
        @Dimensionless
        float theta = Mathematics.ArcCos(cosineTheta);

        // Compute sin(theta)
        @Dimensionless
        float sineTheta = Mathematics.Sin(theta);

        // Compute the two coefficients that are in the spherical linear interpolation formula
        @Dimensionless
        float coeff1 = Mathematics.Sin((((@Dimensionless float) (1.0f)) - t) * theta) / sineTheta;
        @Dimensionless
        float coeff2 = Mathematics.Sin(t * theta) / sineTheta * invert;

        // Compute and return the interpolated quaternion
        quaternionOut.set(oldQuaternion).multiply(coeff1).add(tempQ2.multiply(coeff2));
    }

    @Override
    public @Dimensionless int hashCode(@Dimensionless Quaternion this) {
        @Dimensionless
        int hash = ((@Dimensionless int) (7));
        hash = ((@Dimensionless int) (59)) * hash + Float.floatToIntBits(this.x);
        hash = ((@Dimensionless int) (59)) * hash + Float.floatToIntBits(this.y);
        hash = ((@Dimensionless int) (59)) * hash + Float.floatToIntBits(this.z);
        hash = ((@Dimensionless int) (59)) * hash + Float.floatToIntBits(this.w);
        return hash;
    }

    @Override
    public @Dimensionless boolean equals(@Dimensionless Quaternion this, @Dimensionless Object obj) {

        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final @Dimensionless Quaternion other = (@Dimensionless Quaternion) obj;
        if (Float.floatToIntBits(this.x) != Float.floatToIntBits(other.x)) {
            return false;
        }
        if (Float.floatToIntBits(this.y) != Float.floatToIntBits(other.y)) {
            return false;
        }
        if (Float.floatToIntBits(this.z) != Float.floatToIntBits(other.z)) {
            return false;
        }
        return Float.floatToIntBits(this.w) == Float.floatToIntBits(other.w);
    }

    @Override
    public @Dimensionless String toString(@Dimensionless Quaternion this) {
        return "(w= " + w + ", x= " + x + ", y= " + y + ", z= " + z + ")";
    }

}
