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
package net.smert.jreactphysics3d.engine;

import units.qual.Dimensionless;
import net.smert.jreactphysics3d.body.CollisionBody;
import net.smert.jreactphysics3d.constraint.ContactPoint;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a pair of two bodies that are overlapping during the broad-phase collision detection. It is
 * created when the two bodies start to overlap and is destroyed when they do not overlap anymore. This class contains a
 * contact manifold that store all the contact points between the two bodies.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class OverlappingPair {

    // Pointer to the first body of the contact
    private final @Dimensionless CollisionBody body1;

    // Pointer to the second body of the contact
    private final @Dimensionless CollisionBody body2;

    // Persistent contact manifold
    private final @Dimensionless ContactManifold contactManifold;

    // Cached previous separating axis
    private final @Dimensionless Vector3 cachedSeparatingAxis;

    // Constructor
    public OverlappingPair(@Dimensionless CollisionBody body1, @Dimensionless CollisionBody body2) {
        this.body1 = body1;
        this.body2 = body2;
        contactManifold = new @Dimensionless ContactManifold(body1, body2);
        cachedSeparatingAxis = new @Dimensionless Vector3(((@Dimensionless float) (1.0f)), ((@Dimensionless float) (1.0f)), ((@Dimensionless float) (1.0f)));
    }

    // Add a contact to the contact manifold
    public void addContact(@Dimensionless OverlappingPair this, @Dimensionless ContactPoint contact) {
        contactManifold.addContactPoint(contact);
    }

    // Return the pointer to first body
    public @Dimensionless CollisionBody getBody1(@Dimensionless OverlappingPair this) {
        return body1;
    }

    // Return the pointer to second body
    public @Dimensionless CollisionBody getBody2(@Dimensionless OverlappingPair this) {
        return body2;
    }

    // Return the contact manifold
    public @Dimensionless ContactManifold getContactManifold(@Dimensionless OverlappingPair this) {
        return contactManifold;
    }

    // Return the number of contact points in the contact manifold
    public @Dimensionless int getNumContactPoints(@Dimensionless OverlappingPair this) {
        return contactManifold.getNumContactPoints();
    }

    // Return the cached separating axis
    public @Dimensionless Vector3 getCachedSeparatingAxis(@Dimensionless OverlappingPair this) {
        return cachedSeparatingAxis;
    }

    // Set the cached separating axis
    public void setCachedSeparatingAxis(@Dimensionless OverlappingPair this, @Dimensionless Vector3 axis) {
        cachedSeparatingAxis.set(axis);
    }

    // Update the contact manifold
    public void update(@Dimensionless OverlappingPair this) {
        contactManifold.update(body1.getTransform(), body2.getTransform());
    }

}
