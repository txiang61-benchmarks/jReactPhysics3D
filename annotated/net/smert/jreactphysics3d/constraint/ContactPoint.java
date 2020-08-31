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
 * This class represents a collision contact point between two bodies in the physics engine.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ContactPoint {

    // True if the contact is a resting contact (exists for more than one time step)
    private @Dimensionless boolean isRestingContact;

    // Cached first friction impulse
    private @Dimensionless float frictionImpulse1;

    // Cached second friction impulse
    private @Dimensionless float frictionImpulse2;

    // Penetration depth
    private @Dimensionless float penetrationDepth;

    // Cached penetration impulse
    private @Dimensionless float penetrationImpulse;

    // First rigid body of the contact
    private final @Dimensionless RigidBody body1;

    // Second rigid body of the contact
    private final @Dimensionless RigidBody body2;

    // Contact point on body 1 in local space of body 1
    private final @Dimensionless Vector3 localPointOnBody1;

    // Contact point on body 2 in local space of body 2
    private final @Dimensionless Vector3 localPointOnBody2;

    // Normal vector of the contact (From body1 toward body2) in world space
    private final @Dimensionless Vector3 normal;

    // Contact point on body 1 in world space
    private final @Dimensionless Vector3 worldPointOnBody1;

    // Contact point on body 2 in world space
    private final @Dimensionless Vector3 worldPointOnBody2;

    // Two orthogonal vectors that span the tangential friction plane
    private final @Dimensionless Vector3 @Dimensionless [] frictionVectors = new @Dimensionless Vector3 @Dimensionless [((@Dimensionless int) (2))];

    // Constructor
    public ContactPoint(@Dimensionless ContactPointInfo contactInfo) {
        body1 = contactInfo.getBody1();
        body2 = contactInfo.getBody2();
        normal = contactInfo.getNormal();
        penetrationDepth = contactInfo.getPenetrationDepth();
        localPointOnBody1 = contactInfo.getLocalPoint1();
        localPointOnBody2 = contactInfo.getLocalPoint2();
        worldPointOnBody1 = contactInfo.getBody1().getTransform().multiply(localPointOnBody1, new @Dimensionless Vector3());
        worldPointOnBody2 = contactInfo.getBody2().getTransform().multiply(localPointOnBody2, new @Dimensionless Vector3());
        isRestingContact = false;

        frictionVectors[((@Dimensionless int) (0))] = new @Dimensionless Vector3();
        frictionVectors[((@Dimensionless int) (1))] = new @Dimensionless Vector3();

        assert (penetrationDepth > ((@Dimensionless float) (0.0f)));
    }

    // Return true if the contact is a resting contact
    public @Dimensionless boolean getIsRestingContact(@Dimensionless ContactPoint this) {
        return isRestingContact;
    }

    // Set the mIsRestingContact variable
    public void setIsRestingContact(@Dimensionless ContactPoint this, @Dimensionless boolean isRestingContact) {
        this.isRestingContact = isRestingContact;
    }

    // Return the cached first friction impulse
    public @Dimensionless float getFrictionImpulse1(@Dimensionless ContactPoint this) {
        return frictionImpulse1;
    }

    // Set the first cached friction impulse
    public void setFrictionImpulse1(@Dimensionless ContactPoint this, @Dimensionless float impulse) {
        frictionImpulse1 = impulse;
    }

    // Return the cached second friction impulse
    public @Dimensionless float getFrictionImpulse2(@Dimensionless ContactPoint this) {
        return frictionImpulse2;
    }

    // Set the second cached friction impulse
    public void setFrictionImpulse2(@Dimensionless ContactPoint this, @Dimensionless float impulse) {
        frictionImpulse2 = impulse;
    }

    // Return the penetration depth of the contact
    public float getPenetrationDepth(@Dimensionless ContactPoint this) {
        return penetrationDepth;
    }

    // Set the penetration depth of the contact
    public void setPenetrationDepth(@Dimensionless ContactPoint this, float penetrationDepth) {
        this.penetrationDepth = penetrationDepth;
    }

    // Return the cached penetration impulse
    public @Dimensionless float getPenetrationImpulse(@Dimensionless ContactPoint this) {
        return penetrationImpulse;
    }

    // Set the cached penetration impulse
    public void setPenetrationImpulse(@Dimensionless ContactPoint this, @Dimensionless float impulse) {
        penetrationImpulse = impulse;
    }

    // Return the reference to the body 1
    public @Dimensionless RigidBody getBody1(@Dimensionless ContactPoint this) {
        return body1;
    }

    // Return the reference to the body 2
    public @Dimensionless RigidBody getBody2(@Dimensionless ContactPoint this) {
        return body2;
    }

    // Return the contact point on body 1
    public Vector3 getLocalPointOnBody1(@Dimensionless ContactPoint this) {
        return localPointOnBody1;
    }

    // Return the contact point on body 2
    public Vector3 getLocalPointOnBody2(@Dimensionless ContactPoint this) {
        return localPointOnBody2;
    }

    // Return the normal vector of the contact
    public Vector3 getNormal(@Dimensionless ContactPoint this) {
        return normal;
    }

    // Return the contact world point on body 1
    public Vector3 getWorldPointOnBody1(@Dimensionless ContactPoint this) {
        return worldPointOnBody1;
    }

    // Set the contact world point on body 1
    public void setWorldPointOnBody1(@Dimensionless ContactPoint this, Vector3 worldPoint) {
        worldPointOnBody1.set(worldPoint);
    }

    // Return the contact world point on body 2
    public Vector3 getWorldPointOnBody2(@Dimensionless ContactPoint this) {
        return worldPointOnBody2;
    }

    // Set the contact world point on body 2
    public void setWorldPointOnBody2(@Dimensionless ContactPoint this, Vector3 worldPoint) {
        worldPointOnBody2.set(worldPoint);
    }

    // Get the first friction vector
    public @Dimensionless Vector3 getFrictionVector1(@Dimensionless ContactPoint this) {
        return frictionVectors[((@Dimensionless int) (0))];
    }

    // Set the first friction vector
    public void setFrictionVector1(@Dimensionless ContactPoint this, @Dimensionless Vector3 frictionVector1) {
        frictionVectors[((@Dimensionless int) (0))] = new @Dimensionless Vector3(frictionVector1);
    }

    // Get the second friction vector
    public @Dimensionless Vector3 getFrictionVector2(@Dimensionless ContactPoint this) {
        return frictionVectors[((@Dimensionless int) (1))];
    }

    // Set the second friction vector
    public void setFrictionVector2(@Dimensionless ContactPoint this, @Dimensionless Vector3 frictionVector2) {
        frictionVectors[((@Dimensionless int) (1))] = new @Dimensionless Vector3(frictionVector2);
    }

}
