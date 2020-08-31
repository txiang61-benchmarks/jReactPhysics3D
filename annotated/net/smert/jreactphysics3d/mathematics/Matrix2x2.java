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
 * This class represents a 2x2 matrix.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
@Dimensionless
public class Matrix2x2 {

    // Rows of the matrix (m[row][column])
    @Dimensionless
    float m00;
    @Dimensionless
    float m01;
    @Dimensionless
    float m10;
    @Dimensionless
    float m11;

    // Constructor
    public Matrix2x2() {
        zero();
    }

    // Constructor
    public Matrix2x2(@Dimensionless float value) {
        set(value, value, value, value);
    }

    // Constructor with arguments
    public Matrix2x2(@Dimensionless float a1, @Dimensionless float a2, @Dimensionless float b1, @Dimensionless float b2) {
        set(a1, a2, b1, b2);
    }

    // Copy-constructor
    public Matrix2x2(@Dimensionless Matrix2x2 matrix) {
        set(matrix);
    }

    // Return the determinant of the matrix
    public @Dimensionless float getDeterminant(@Dimensionless Matrix2x2 this) {
        return m00 * m11 - m10 * m01;
    }

    // Return the trace of the matrix
    public @Dimensionless float getTrace(@Dimensionless Matrix2x2 this) {
        return m00 + m11;
    }

    // Return the matrix with absolute values
    public @Dimensionless Matrix2x2 abs(@Dimensionless Matrix2x2 this) {
        m00 = Math.abs(m00);
        m01 = Math.abs(m01);
        m10 = Math.abs(m10);
        m11 = Math.abs(m11);
        return this;
    }

    // Overloaded operator for addition with assignment
    public @Dimensionless Matrix2x2 add(@Dimensionless Matrix2x2 this, @Dimensionless Matrix2x2 matrix) {
        m00 += matrix.m00;
        m01 += matrix.m01;
        m10 += matrix.m10;
        m11 += matrix.m11;
        return this;
    }

    // Set the matrix to the identity matrix
    public @Dimensionless Matrix2x2 identity(@Dimensionless Matrix2x2 this) {
        m00 = ((@Dimensionless float) (1.0f));
        m01 = ((@Dimensionless float) (0.0f));
        m10 = ((@Dimensionless float) (0.0f));
        m11 = ((@Dimensionless float) (1.0f));
        return this;
    }

    // Return the inverse matrix
    public @Dimensionless Matrix2x2 inverse(@Dimensionless Matrix2x2 this) {

        // Compute the determinant of the matrix
        @Dimensionless
        float determinant = getDeterminant();

        // Check if the determinant is equal to zero
        assert (Math.abs(determinant) > Defaults.MACHINE_EPSILON);
        @Dimensionless
        float invDeterminant = ((@Dimensionless float) (1.0f)) / determinant;

        set(m11, -m01, -m10, m00);

        // Return the inverse matrix
        return multiply(invDeterminant);
    }

    // Overloaded operator for the negative of the matrix
    public @Dimensionless Matrix2x2 invert(@Dimensionless Matrix2x2 this) {
        m00 = -m00;
        m01 = -m01;
        m10 = -m10;
        m11 = -m11;
        return this;
    }

    // Overloaded operator for multiplication with a number with assignment
    public @Dimensionless Matrix2x2 multiply(@Dimensionless Matrix2x2 this, @Dimensionless float number) {
        m00 *= number;
        m01 *= number;
        m10 *= number;
        m11 *= number;
        return this;
    }

    // Overloaded operator for matrix multiplication
    public @Dimensionless Matrix2x2 multiply(@Dimensionless Matrix2x2 this, @Dimensionless Matrix2x2 matrix) {
        set(
                m00 * matrix.m00 + m01 * matrix.m10,
                m00 * matrix.m01 + m01 * matrix.m11,
                m10 * matrix.m00 + m11 * matrix.m10,
                m10 * matrix.m01 + m11 * matrix.m11);
        return this;
    }

    // Method to set all the values in the matrix
    public final @Dimensionless Matrix2x2 set(@Dimensionless Matrix2x2 this, @Dimensionless float a1, @Dimensionless float a2, @Dimensionless float b1, @Dimensionless float b2) {
        m00 = a1;
        m01 = a2;
        m10 = b1;
        m11 = b2;
        return this;
    }

    public final @Dimensionless Matrix2x2 set(@Dimensionless Matrix2x2 this, @Dimensionless Matrix2x2 matrix) {
        m00 = matrix.m00;
        m01 = matrix.m01;
        m10 = matrix.m10;
        m11 = matrix.m11;
        return this;
    }

    // Overloaded operator for substraction with assignment
    public @Dimensionless Matrix2x2 subtract(@Dimensionless Matrix2x2 this, @Dimensionless Matrix2x2 matrix) {
        m00 -= matrix.m00;
        m01 -= matrix.m01;
        m10 -= matrix.m10;
        m11 -= matrix.m11;
        return this;
    }

    // Return the transpose matrix
    public @Dimensionless Matrix2x2 transpose(@Dimensionless Matrix2x2 this) {
        set(m00, m10, m01, m11);
        return this;
    }

    // Set the matrix to zero
    public final @Dimensionless Matrix2x2 zero(@Dimensionless Matrix2x2 this) {
        m00 = ((@Dimensionless float) (0.0f));
        m01 = ((@Dimensionless float) (0.0f));
        m10 = ((@Dimensionless float) (0.0f));
        m11 = ((@Dimensionless float) (0.0f));
        return this;
    }

    // Return a column
    public @Dimensionless Vector2 getColumn(@Dimensionless Matrix2x2 this, @Dimensionless int index) {
        if (index == ((@Dimensionless int) (0))) {
            return new @Dimensionless Vector2(m00, m10);
        } else if (index == ((@Dimensionless int) (1))) {
            return new @Dimensionless Vector2(m01, m11);
        }
        throw new @Dimensionless IllegalArgumentException("Unknown column index: " + index);
    }

    // Return a row
    public @Dimensionless Vector2 getRow(@Dimensionless Matrix2x2 this, @Dimensionless int index) {
        if (index == ((@Dimensionless int) (0))) {
            return new @Dimensionless Vector2(m00, m01);
        } else if (index == ((@Dimensionless int) (1))) {
            return new @Dimensionless Vector2(m10, m11);
        }
        throw new @Dimensionless IllegalArgumentException("Unknown row index: " + index);
    }

    // Overloaded operator for multiplication with a vector
    public @Dimensionless Vector2 multiply(@Dimensionless Matrix2x2 this, @Dimensionless Vector2 vector, @Dimensionless Vector2 vectorOut) {
        return vectorOut.set(
                m00 * vector.x + m01 * vector.y,
                m10 * vector.x + m11 * vector.y);
    }

    @Override
    public @Dimensionless int hashCode(@Dimensionless Matrix2x2 this) {
        @Dimensionless
        int hash = ((@Dimensionless int) (5));
        hash = ((@Dimensionless int) (23)) * hash + Float.floatToIntBits(this.m00);
        hash = ((@Dimensionless int) (23)) * hash + Float.floatToIntBits(this.m10);
        hash = ((@Dimensionless int) (23)) * hash + Float.floatToIntBits(this.m01);
        hash = ((@Dimensionless int) (23)) * hash + Float.floatToIntBits(this.m11);
        return hash;
    }

    @Override
    public @Dimensionless boolean equals(@Dimensionless Matrix2x2 this, @Dimensionless Object obj) {

        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final @Dimensionless Matrix2x2 other = (@Dimensionless Matrix2x2) obj;
        if (Float.floatToIntBits(this.m00) != Float.floatToIntBits(other.m00)) {
            return false;
        }
        if (Float.floatToIntBits(this.m10) != Float.floatToIntBits(other.m10)) {
            return false;
        }
        if (Float.floatToIntBits(this.m01) != Float.floatToIntBits(other.m01)) {
            return false;
        }
        return Float.floatToIntBits(this.m11) == Float.floatToIntBits(other.m11);
    }

    @Override
    public @Dimensionless String toString(@Dimensionless Matrix2x2 this) {
        return "(00= " + m00 + ", 01= " + m01
                + ", 10= " + m10 + ", 11= " + m11 + ")";
    }

}
