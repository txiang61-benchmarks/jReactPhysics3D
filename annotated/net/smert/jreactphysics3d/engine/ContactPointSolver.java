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
import net.smert.jreactphysics3d.constraint.ContactPoint;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * Contact solver internal data structure that to store all the information relative to a contact point
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ContactPointSolver {

    // True if the contact was existing last time step
    public @Dimensionless boolean isRestingContact;

    // Accumulated impulse in the 1st friction direction
    public @Dimensionless float friction1Impulse;

    // Accumulated impulse in the 2nd friction direction
    public @Dimensionless float friction2Impulse;

    // Inverse of the matrix K for the 1st friction
    public @Dimensionless float inverseFriction1Mass;

    // Inverse of the matrix K for the 2nd friction
    public @Dimensionless float inverseFriction2Mass;

    // Inverse of the matrix K for the penenetration
    public @Dimensionless float inversePenetrationMass;

    // Penetration depth
    public @Dimensionless float penetrationDepth;

    // Accumulated normal impulse
    public @Dimensionless float penetrationImpulse;

    // Accumulated split impulse for penetration correction
    public @Dimensionless float penetrationSplitImpulse;

    // Velocity restitution bias
    public @Dimensionless float restitutionBias;

    // Pointer to the external contact
    public @Dimensionless ContactPoint externalContact;

    // First friction vector in the tangent plane
    public final @Dimensionless Vector3 frictionVector1 = new @Dimensionless Vector3();

    // Second friction vector in the tangent plane
    public final @Dimensionless Vector3 frictionVector2 = new @Dimensionless Vector3();

    // Normal vector of the contact
    public final @Dimensionless Vector3 normal = new @Dimensionless Vector3();

    // Old first friction vector in the tangent plane
    public final @Dimensionless Vector3 oldFrictionVector1 = new @Dimensionless Vector3();

    // Old second friction vector in the tangent plane
    public final @Dimensionless Vector3 oldFrictionVector2 = new @Dimensionless Vector3();

    // Vector from the body 1 center to the contact point
    public final @Dimensionless Vector3 r1 = new @Dimensionless Vector3();

    // Vector from the body 2 center to the contact point
    public final @Dimensionless Vector3 r2 = new @Dimensionless Vector3();

    // Cross product of r1 with the contact normal
    public final @Dimensionless Vector3 r1CrossN = new @Dimensionless Vector3();

    // Cross product of r2 with the contact normal
    public final @Dimensionless Vector3 r2CrossN = new @Dimensionless Vector3();

    // Cross product of r1 with 1st friction vector
    public final @Dimensionless Vector3 r1CrossT1 = new @Dimensionless Vector3();

    // Cross product of r1 with 2nd friction vector
    public final @Dimensionless Vector3 r1CrossT2 = new @Dimensionless Vector3();

    // Cross product of r2 with 1st friction vector
    public final @Dimensionless Vector3 r2CrossT1 = new @Dimensionless Vector3();

    // Cross product of r2 with 2nd friction vector
    public final @Dimensionless Vector3 r2CrossT2 = new @Dimensionless Vector3();

}
