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
 * This class represents a 2D vector.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class Vector2 {

    // Component x
    float x;

    // Component y
    float y;

    // Constructor
    public Vector2() {
        zero();
    }

    // Constructor with arguments
    public Vector2(float x, float y) {
        set(x, y);
    }

    // Copy-constructor
    public Vector2(@Dimensionless Vector2 vector) {
        set(vector);
    }

    // Return true if the vector is unit and false otherwise
    public @Dimensionless boolean isUnit(@Dimensionless Vector2 this) {
        return Mathematics.ApproxEqual(lengthSquare(), ((@Dimensionless float) (1.0f)), Defaults.MACHINE_EPSILON);
    }

    // Return true if the vector is the zero vector
    public @Dimensionless boolean isZero(@Dimensionless Vector2 this) {
        return Mathematics.ApproxEqual(lengthSquare(), ((@Dimensionless float) (0.0f)), Defaults.MACHINE_EPSILON);
    }

    // Scalar product of two vectors (public)
    public @Dimensionless float dot(@Dimensionless Vector2 this, @Dimensionless Vector2 vector) {
        return x * vector.x + y * vector.y;
    }

    // Overloaded operator for value access
    public @Dimensionless float get(@Dimensionless Vector2 this, @Dimensionless int index) {
        if (index == ((@Dimensionless int) (0))) {
            return x;
        } else if (index == ((@Dimensionless int) (1))) {
            return y;
        }
        throw new @Dimensionless IllegalArgumentException("Unknown index: " + index);
    }

    public @Dimensionless float getX(@Dimensionless Vector2 this) {
        return x;
    }

    public @Dimensionless float getY(@Dimensionless Vector2 this) {
        return y;
    }

    // Return the length of the vector
    public @Dimensionless float length(@Dimensionless Vector2 this) {
        return Mathematics.Sqrt(x * x + y * y);
    }

    // Return the square of the length of the vector
    public @Dimensionless float lengthSquare(@Dimensionless Vector2 this) {
        return x * x + y * y;
    }

    // Return the axis with the maximal value
    public @Dimensionless int getMaxAxis(@Dimensionless Vector2 this) {
        return (x < y ? ((@Dimensionless int) (1)) : ((@Dimensionless int) (0)));
    }

    // Return the axis with the minimal value
    public @Dimensionless int getMinAxis(@Dimensionless Vector2 this) {
        return (x < y ? ((@Dimensionless int) (0)) : ((@Dimensionless int) (1)));
    }

    // Return the corresponding absolute value vector
    public @Dimensionless Vector2 abs(@Dimensionless Vector2 this) {
        x = Math.abs(x);
        y = Math.abs(y);
        return this;
    }

    // Overloaded operator for addition with assignment
    public @Dimensionless Vector2 add(@Dimensionless Vector2 this, @Dimensionless Vector2 vector) {
        x += vector.x;
        y += vector.y;
        return this;
    }

    // Overloaded operator for division by a number with assignment
    public @Dimensionless Vector2 divide(@Dimensionless Vector2 this, @Dimensionless float number) {
        assert (number > Defaults.MACHINE_EPSILON);
        x /= number;
        y /= number;
        return this;
    }

    // Overloaded operator for the negative of a vector
    public @Dimensionless Vector2 invert(@Dimensionless Vector2 this) {
        x = -x;
        y = -y;
        return this;
    }

    // Overloaded operator for multiplication with a number with assignment
    public @Dimensionless Vector2 multiply(@Dimensionless Vector2 this, @Dimensionless float number) {
        x *= number;
        y *= number;
        return this;
    }

    // Normalize the vector
    public @Dimensionless Vector2 normalize(@Dimensionless Vector2 this) {
        @Dimensionless
        float len = length();
        assert (len > Defaults.MACHINE_EPSILON);
        x /= len;
        y /= len;
        return this;
    }

    // Set all the values of the vector
    public final Vector2 set(@Dimensionless Vector2 this, float x, float y) {
        this.x = x;
        this.y = y;
        return this;
    }

    // Assignment operator
    public final @Dimensionless Vector2 set(@Dimensionless Vector2 this, @Dimensionless Vector2 vector) {
        x = vector.x;
        y = vector.y;
        return this;
    }

    // Return one unit orthogonal vector of the current vector
    public @Dimensionless Vector2 setUnitOrthogonal(@Dimensionless Vector2 this) {
        @Dimensionless
        float len = length();
        assert (len > Defaults.MACHINE_EPSILON);
        return set(-y / len, x / len);
    }

    public @Dimensionless Vector2 setX(@Dimensionless Vector2 this, @Dimensionless float x) {
        this.x = x;
        return this;
    }

    public @Dimensionless Vector2 setY(@Dimensionless Vector2 this, @Dimensionless float y) {
        this.y = y;
        return this;
    }

    // Overloaded operator for substraction with assignment
    public @Dimensionless Vector2 subtract(@Dimensionless Vector2 this, @Dimensionless Vector2 vector) {
        x -= vector.x;
        y -= vector.y;
        return this;
    }

    // Set the vector to zero
    public final @Dimensionless Vector2 zero(@Dimensionless Vector2 this) {
        x = ((@Dimensionless float) (0.0f));
        y = ((@Dimensionless float) (0.0f));
        return this;
    }

    @Override
    public @Dimensionless int hashCode(@Dimensionless Vector2 this) {
        @Dimensionless
        int hash = ((@Dimensionless int) (3));
        hash = ((@Dimensionless int) (53)) * hash + Float.floatToIntBits(this.x);
        hash = ((@Dimensionless int) (53)) * hash + Float.floatToIntBits(this.y);
        return hash;
    }

    @Override
    public @Dimensionless boolean equals(@Dimensionless Vector2 this, @Dimensionless Object obj) {

        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final @Dimensionless Vector2 other = (@Dimensionless Vector2) obj;
        if (Float.floatToIntBits(this.x) != Float.floatToIntBits(other.x)) {
            return false;
        }
        return Float.floatToIntBits(this.y) == Float.floatToIntBits(other.y);
    }

    @Override
    public @Dimensionless String toString(@Dimensionless Vector2 this) {
        return "(x= " + x + ", y= " + y + ")";
    }

}
