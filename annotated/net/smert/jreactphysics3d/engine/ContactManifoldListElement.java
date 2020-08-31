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

/**
 * This structure represents a single element of a linked list of contact manifolds
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class ContactManifoldListElement {

    // Pointer to the actual contact manifold
    private final @Dimensionless ContactManifold contactManifold;

    // Next element of the list
    private final @Dimensionless ContactManifoldListElement next;

    // Constructor
    public ContactManifoldListElement(@Dimensionless ContactManifold contactManifold, @Dimensionless ContactManifoldListElement next) {
        this.contactManifold = contactManifold;
        this.next = next;
    }

    public @Dimensionless ContactManifold getContactManifold(@Dimensionless ContactManifoldListElement this) {
        return contactManifold;
    }

    public @Dimensionless ContactManifoldListElement getNext(@Dimensionless ContactManifoldListElement this) {
        return next;
    }

}
