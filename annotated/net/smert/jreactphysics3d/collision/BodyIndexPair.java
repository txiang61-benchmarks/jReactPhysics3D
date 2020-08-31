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
package net.smert.jreactphysics3d.collision;
import units.qual.Dimensionless;

/**
 * typedef std::pair<bodyindex, bodyindex> bodyindexpair;
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class BodyIndexPair {

    private final @Dimensionless int first;
    private final @Dimensionless int second;

    public BodyIndexPair(int first, int second) {
        this.first = first;
        this.second = second;
    }

    public int getFirst(@Dimensionless BodyIndexPair this) {
        return first;
    }

    public int getSecond(@Dimensionless BodyIndexPair this) {
        return second;
    }

    @Override
    public @Dimensionless int hashCode(@Dimensionless BodyIndexPair this) {
        @Dimensionless
        int hash = ((@Dimensionless int) (7));
        hash = ((@Dimensionless int) (29)) * hash + this.first;
        hash = ((@Dimensionless int) (29)) * hash + this.second;
        return hash;
    }

    @Override
    public @Dimensionless boolean equals(@Dimensionless BodyIndexPair this, @Dimensionless Object obj) {
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final @Dimensionless BodyIndexPair other = (@Dimensionless BodyIndexPair) obj;
        if (this.first != other.first) {
            return false;
        }
        return this.second == other.second;
    }

}
