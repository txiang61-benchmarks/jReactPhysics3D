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
 * This class represents a 3D vector.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Vector3 {

    // Component x
    @Dimensionless
    float x;

    // Component y
    @Dimensionless
    float y;

    // Component z
    @Dimensionless
    float z;

    // Constructor
    public Vector3() {
        zero();
    }

    // Constructor with arguments
    public Vector3(@Dimensionless float x, @Dimensionless float y, @Dimensionless float z) {
        set(x, y, z);
    }

    // Copy-constructor
    public Vector3(Vector3 vector) {
        set(vector);
    }

    // Return true if the vector is unit and false otherwise
    public @Dimensionless boolean isUnit(@Dimensionless Vector3 this) {
        return Mathematics.ApproxEqual(lengthSquare(), ((@Dimensionless float) (1.0f)), Defaults.MACHINE_EPSILON);
    }

    // Return true if the vector is the zero vector
    public @Dimensionless boolean isZero(@Dimensionless Vector3 this) {
        return Mathematics.ApproxEqual(lengthSquare(), ((@Dimensionless float) (0.0f)), Defaults.MACHINE_EPSILON);
    }

    // Scalar product of two vectors (public)
    public @Dimensionless float dot(@Dimensionless Vector3 this, @Dimensionless Vector3 vector) {
        return x * vector.x + y * vector.y + z * vector.z;
    }

    // Overloaded operator for value access
    public @Dimensionless float get(@Dimensionless Vector3 this, @Dimensionless int index) {
        if (index == ((@Dimensionless int) (0))) {
            return x;
        } else if (index == ((@Dimensionless int) (1))) {
            return y;
        } else if (index == ((@Dimensionless int) (2))) {
            return z;
        }
        throw new @Dimensionless IllegalArgumentException("Unknown index: " + index);
    }

    public @Dimensionless float getX(@Dimensionless Vector3 this) {
        return x;
    }

    public @Dimensionless float getY(@Dimensionless Vector3 this) {
        return y;
    }

    public @Dimensionless float getZ(@Dimensionless Vector3 this) {
        return z;
    }

    // Return the length of the vector
    public @Dimensionless float length(@Dimensionless Vector3 this) {
        return Mathematics.Sqrt(x * x + y * y + z * z);
    }

    // Return the square of the length of the vector
    public @Dimensionless float lengthSquare(@Dimensionless Vector3 this) {
        return x * x + y * y + z * z;
    }

    // Return the axis with the maximal value
    public @Dimensionless int getMaxAxis(@Dimensionless Vector3 this) {
        return (x < y ? (y < z ? ((@Dimensionless int) (2)) : ((@Dimensionless int) (1))) : (x < z ? ((@Dimensionless int) (2)) : ((@Dimensionless int) (0))));
    }

    // Return the axis with the minimal value
    public @Dimensionless int getMinAxis(@Dimensionless Vector3 this) {
        return (x < y ? (x < z ? ((@Dimensionless int) (0)) : ((@Dimensionless int) (2))) : (y < z ? ((@Dimensionless int) (1)) : ((@Dimensionless int) (2))));
    }

    // Return the corresponding absolute value vector
    public @Dimensionless Vector3 abs(@Dimensionless Vector3 this) {
        x = Math.abs(x);
        y = Math.abs(y);
        z = Math.abs(z);
        return this;
    }

    // Overloaded operator for addition with assignment
    public Vector3 add(@Dimensionless Vector3 this, Vector3 vector) {
        x += vector.x;
        y += vector.y;
        z += vector.z;
        return this;
    }

    // Cross product of two vectors (public)
    public Vector3 cross(@Dimensionless Vector3 this, Vector3 vector) {
        return set(
                y * vector.z - z * vector.y,
                z * vector.x - x * vector.z,
                x * vector.y - y * vector.x);
    }

    // Overloaded operator for division by a number with assignment
    public @Dimensionless Vector3 divide(@Dimensionless Vector3 this, @Dimensionless float number) {
        assert (number > Defaults.MACHINE_EPSILON);
        x /= number;
        y /= number;
        z /= number;
        return this;
    }

    // Overloaded operator for the negative of a vector
    public @Dimensionless Vector3 invert(@Dimensionless Vector3 this) {
        x = -x;
        y = -y;
        z = -z;
        return this;
    }

    // Overloaded operator for multiplication with a number with assignment
    public @Dimensionless Vector3 multiply(@Dimensionless Vector3 this, @Dimensionless float number) {
        x *= number;
        y *= number;
        z *= number;
        return this;
    }

    // Normalize the vector
    public @Dimensionless Vector3 normalize(@Dimensionless Vector3 this) {
        @Dimensionless
        float len = length();
        assert (len > Defaults.MACHINE_EPSILON);
        @Dimensionless
        float lenInv = ((@Dimensionless float) (1.0f)) / len;
        x *= lenInv;
        y *= lenInv;
        z *= lenInv;
        return this;
    }

    // Set all the values of the vector
    public final @Dimensionless Vector3 set(@Dimensionless Vector3 this, @Dimensionless float x, @Dimensionless float y, @Dimensionless float z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }

    // Assignment operator
    public final Vector3 set(@Dimensionless Vector3 this, Vector3 vector) {
        x = vector.x;
        y = vector.y;
        z = vector.z;
        return this;
    }

    // Return one unit orthogonal vector of the current vector
    public @Dimensionless Vector3 setUnitOrthogonal(@Dimensionless Vector3 this) {

        float len, lenInv;

        // Get the minimum element of the vector
        @Dimensionless
        Vector3 abs = new @Dimensionless Vector3(this).abs();
        @Dimensionless
        int minElement = abs.getMinAxis();

        if (minElement == ((@Dimensionless int) (0))) {
            len = Mathematics.Sqrt(y * y + z * z);
            lenInv = ((@Dimensionless float) (1.0f)) / len;
            set(((@Dimensionless float) (0.0f)), -z, y).multiply(lenInv);
        } else if (minElement == ((@Dimensionless int) (1))) {
            len = Mathematics.Sqrt(x * x + z * z);
            lenInv = ((@Dimensionless float) (1.0f)) / len;
            set(-z, ((@Dimensionless float) (0.0f)), x).multiply(lenInv);
        } else {
            len = Mathematics.Sqrt(x * x + y * y);
            lenInv = ((@Dimensionless float) (1.0f)) / len;
            set(-y, x, ((@Dimensionless float) (0.0f))).multiply(lenInv);
        }

        return this;
    }

    public @Dimensionless Vector3 setX(@Dimensionless Vector3 this, @Dimensionless float x) {
        this.x = x;
        return this;
    }

    public @Dimensionless Vector3 setY(@Dimensionless Vector3 this, @Dimensionless float y) {
        this.y = y;
        return this;
    }

    public @Dimensionless Vector3 setZ(@Dimensionless Vector3 this, @Dimensionless float z) {
        this.z = z;
        return this;
    }

    // Overloaded operator for substraction with assignment
    public Vector3 subtract(@Dimensionless Vector3 this, Vector3 vector) {
        x -= vector.x;
        y -= vector.y;
        z -= vector.z;
        return this;
    }

    // Set the vector to zero
    public final Vector3 zero(@Dimensionless Vector3 this) {
        x = ((@Dimensionless float) (0.0f));
        y = ((@Dimensionless float) (0.0f));
        z = ((@Dimensionless float) (0.0f));
        return this;
    }

    public static void Lerp(@Dimensionless Vector3 oldVector, @Dimensionless Vector3 newVector, @Dimensionless float t, @Dimensionless Vector3 vectorOut) {
        assert (t >= ((@Dimensionless float) (0.0f)) && t <= ((@Dimensionless float) (1.0f)));
        vectorOut.set(oldVector).multiply(((@Dimensionless float) (1.0f)) - t).add(new @Dimensionless Vector3(newVector).multiply(t));
    }

    @Override
    public @Dimensionless int hashCode(@Dimensionless Vector3 this) {
        @Dimensionless
        int hash = ((@Dimensionless int) (5));
        hash = ((@Dimensionless int) (11)) * hash + Float.floatToIntBits(this.x);
        hash = ((@Dimensionless int) (11)) * hash + Float.floatToIntBits(this.y);
        hash = ((@Dimensionless int) (11)) * hash + Float.floatToIntBits(this.z);
        return hash;
    }

    @Override
    public @Dimensionless boolean equals(@Dimensionless Vector3 this, @Dimensionless Object obj) {

        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final @Dimensionless Vector3 other = (@Dimensionless Vector3) obj;
        if (Float.floatToIntBits(this.x) != Float.floatToIntBits(other.x)) {
            return false;
        }
        if (Float.floatToIntBits(this.y) != Float.floatToIntBits(other.y)) {
            return false;
        }
        return Float.floatToIntBits(this.z) == Float.floatToIntBits(other.z);
    }

    @Override
    public @Dimensionless String toString(@Dimensionless Vector3 this) {
        return "(x= " + x + ", y= " + y + ", z= " + z + ")";
    }

}
