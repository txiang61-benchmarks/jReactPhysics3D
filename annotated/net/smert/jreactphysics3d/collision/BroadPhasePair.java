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
import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This structure represents a pair of bodies during the broad-phase collision detection.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class BroadPhasePair {

    // Pointer to the first body
    private final @Dimensionless CollisionBody body1;

    // Pointer to the second body
    private final @Dimensionless CollisionBody body2;

    // Previous cached separating axis
    private final @Dimensionless Vector3 previousSeparatingAxis;

    // Constructor
    public BroadPhasePair(@Dimensionless CollisionBody body1, @Dimensionless CollisionBody body2) {
        this.body1 = body1;
        this.body2 = body2;
        previousSeparatingAxis = new @Dimensionless Vector3(((@Dimensionless float) (1.0f)), ((@Dimensionless float) (1.0f)), ((@Dimensionless float) (1.0f)));
    }

    public @Dimensionless CollisionBody getBody1(@Dimensionless BroadPhasePair this) {
        return body1;
    }

    public @Dimensionless CollisionBody getBody2(@Dimensionless BroadPhasePair this) {
        return body2;
    }

    public @Dimensionless Vector3 getPreviousSeparatingAxis(@Dimensionless BroadPhasePair this) {
        return previousSeparatingAxis;
    }

    // Return the pair of bodies index
    public @Dimensionless BodyIndexPair newBodiesIndexPair(@Dimensionless BroadPhasePair this) {
        return ComputeBodiesIndexPair(body1, body2);
    }

    // Return the pair of bodies index
    public static @Dimensionless BodyIndexPair ComputeBodiesIndexPair(@Dimensionless CollisionBody body1, @Dimensionless CollisionBody body2) {

        // Construct the pair of body index
        @Dimensionless
        BodyIndexPair indexPair = body1.getBodyID() < body2.getBodyID()
                ? new @Dimensionless BodyIndexPair(body1.getBodyID(), body2.getBodyID())
                : new @Dimensionless BodyIndexPair(body2.getBodyID(), body1.getBodyID());
        assert (indexPair.getFirst() != indexPair.getSecond());
        return indexPair;
    }

}
