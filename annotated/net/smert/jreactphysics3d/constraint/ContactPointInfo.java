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
package net.smert.jreactphysics3d.constraint;

import units.qual.Dimensionless;
import net.smert.jreactphysics3d.body.RigidBody;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This structure contains informations about a collision contact computed during the narrow-phase collision detection.
 * Those informations are used to compute the contact set for a contact between two bodies.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ContactPointInfo {

    // Penetration depth of the contact
    private @Dimensionless float penetrationDepth;

    // First rigid body of the constraint
    private @Dimensionless RigidBody body1;

    // Second rigid body of the constraint
    private @Dimensionless RigidBody body2;

    // Normal vector the the collision contact in world space
    private final @Dimensionless Vector3 normal;

    // Contact point of body 1 in local space of body 1
    private final @Dimensionless Vector3 localPoint1;

    // Contact point of body 2 in local space of body 2
    private final @Dimensionless Vector3 localPoint2;

    // Constructor
    public ContactPointInfo() {
        this.normal = new @Dimensionless Vector3();
        this.penetrationDepth = ((@Dimensionless float) (0.0f));
        this.localPoint1 = new @Dimensionless Vector3();
        this.localPoint2 = new @Dimensionless Vector3();
    }

    public @Dimensionless float getPenetrationDepth(@Dimensionless ContactPointInfo this) {
        return penetrationDepth;
    }

    public @Dimensionless RigidBody getBody1(@Dimensionless ContactPointInfo this) {
        return body1;
    }

    public void setBody1(@Dimensionless ContactPointInfo this, @Dimensionless RigidBody body1) {
        this.body1 = body1;
    }

    public @Dimensionless RigidBody getBody2(@Dimensionless ContactPointInfo this) {
        return body2;
    }

    public void setBody2(@Dimensionless ContactPointInfo this, @Dimensionless RigidBody body2) {
        this.body2 = body2;
    }

    public @Dimensionless Vector3 getNormal(@Dimensionless ContactPointInfo this) {
        return normal;
    }

    public @Dimensionless Vector3 getLocalPoint1(@Dimensionless ContactPointInfo this) {
        return localPoint1;
    }

    public @Dimensionless Vector3 getLocalPoint2(@Dimensionless ContactPointInfo this) {
        return localPoint2;
    }

    public void setCollisionData(@Dimensionless ContactPointInfo this, @Dimensionless Vector3 normal, @Dimensionless float penetrationDepth, @Dimensionless Vector3 localPoint1, @Dimensionless Vector3 localPoint2) {
        this.normal.set(normal);
        this.penetrationDepth = penetrationDepth;
        this.localPoint1.set(localPoint1);
        this.localPoint2.set(localPoint2);
    }

}
