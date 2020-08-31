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
package net.smert.jreactphysics3d.body;

import units.qual.Dimensionless;
import net.smert.jreactphysics3d.collision.shapes.CollisionShape;
import net.smert.jreactphysics3d.constraint.Joint;
import net.smert.jreactphysics3d.constraint.JointListElement;
import net.smert.jreactphysics3d.engine.Material;
import net.smert.jreactphysics3d.mathematics.Matrix3x3;
import net.smert.jreactphysics3d.mathematics.Transform;
import net.smert.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a rigid body of the physics engine. A rigid body is a non-deformable body that has a constant
 * mass. This class inherits from the CollisionBody class.
 *
 * @author Jason Sorensen <sorensenj@smert.net>
 */
public class RigidBody extends CollisionBody {

    // True if the gravity needs to be applied to this rigid body
    protected @Dimensionless boolean isGravityEnabled;

    // Angular velocity damping factor
    protected @Dimensionless float angularDamping;

    // Linear velocity damping factor
    protected @Dimensionless float linearDamping;

    // TODO: Remove the mass variable (duplicate with inverseMass)
    // Mass of the body
    protected @Dimensionless float mass;

    // Inverse of the mass of the body
    protected @Dimensionless float massInverse;

    // First element of the linked list of joints involving this body
    protected @Dimensionless JointListElement jointsList;

    // Angular velocity of the body
    protected final @Dimensionless Vector3 angularVelocity;

    // Linear velocity of the body
    protected final @Dimensionless Vector3 linearVelocity;

    // Current external force on the body
    protected final @Dimensionless Vector3 externalForce;

    // Current external torque on the body
    protected final @Dimensionless Vector3 externalTorque;

    // Material properties of the rigid body
    protected final @Dimensionless Material material;

    // Local inertia tensor of the body (in local-space)
    protected final @Dimensionless Matrix3x3 inertiaTensorLocal;

    // Inverse of the inertia tensor of the body
    protected final @Dimensionless Matrix3x3 inertiaTensorLocalInverse;

    // Constructor
    public RigidBody(@Dimensionless Transform transform, @Dimensionless float mass, @Dimensionless Matrix3x3 inertiaTensorLocal, @Dimensionless CollisionShape collisionShape, @Dimensionless int id) {
        super(transform, collisionShape, id);

        assert (mass > ((@Dimensionless float) (0.0f)));
        assert (inertiaTensorLocal != null);

        isGravityEnabled = true;
        linearDamping = ((@Dimensionless float) (0.0f));
        angularDamping = ((@Dimensionless float) (0.0f));
        this.mass = mass;
        massInverse = ((@Dimensionless float) (1.0f)) / mass;

        jointsList = null;
        angularVelocity = new @Dimensionless Vector3();
        linearVelocity = new @Dimensionless Vector3();
        externalForce = new @Dimensionless Vector3();
        externalTorque = new @Dimensionless Vector3();
        material = new @Dimensionless Material();
        this.inertiaTensorLocal = new @Dimensionless Matrix3x3(inertiaTensorLocal);
        inertiaTensorLocalInverse = new @Dimensionless Matrix3x3(inertiaTensorLocal).inverse();
    }

    // Return true if the gravity needs to be applied to this rigid body
    public @Dimensionless boolean isGravityEnabled(@Dimensionless RigidBody this) {
        return isGravityEnabled;
    }

    // Set the variable to know if the gravity is applied to this rigid body
    public void enableGravity(@Dimensionless RigidBody this, @Dimensionless boolean isEnabled) {
        isGravityEnabled = isEnabled;
    }

    // Return the angular velocity damping factor
    public @Dimensionless float getAngularDamping(@Dimensionless RigidBody this) {
        return angularDamping;
    }

    // Set the angular damping factor
    public void setAngularDamping(@Dimensionless RigidBody this, @Dimensionless float angularDamping) {
        assert (angularDamping >= ((@Dimensionless float) (0.0f)));
        this.angularDamping = angularDamping;
    }

    // Return the linear velocity damping factor
    public @Dimensionless float getLinearDamping(@Dimensionless RigidBody this) {
        return linearDamping;
    }

    // Set the linear damping factor
    public void setLinearDamping(@Dimensionless RigidBody this, @Dimensionless float linearDamping) {
        assert (linearDamping >= ((@Dimensionless float) (0.0f)));
        this.linearDamping = linearDamping;
    }

    // Method that return the mass of the body
    public @Dimensionless float getMass(@Dimensionless RigidBody this) {
        return mass;
    }

    // Method that set the mass of the body
    public void setMass(@Dimensionless RigidBody this, @Dimensionless float mass) {
        assert (mass > ((@Dimensionless float) (0.0f)));

        // TODO: Set inverse mass when this is set?
        this.mass = mass;
    }

    // Return the inverse of the mass of the body
    public @Dimensionless float getMassInverse(@Dimensionless RigidBody this) {
        return massInverse;
    }

    // Set the inverse of the mass
    public void setMassInverse(@Dimensionless RigidBody this, @Dimensionless float massInverse) {
        assert (massInverse >= ((@Dimensionless float) (0.0f)));
        this.massInverse = massInverse;
    }

    // Return the first element of the linked list of joints involving this body
    public @Dimensionless JointListElement getJointsList(@Dimensionless RigidBody this) {
        return jointsList;
    }

    public void setJointsList(@Dimensionless RigidBody this, @Dimensionless JointListElement jointsList) {
        this.jointsList = jointsList;
    }

    // Remove a joint from the joints list
    public void removeJointFromJointsList(@Dimensionless RigidBody this, @Dimensionless Joint joint) {

        assert (joint != null);
        assert (jointsList != null);

        // Remove the joint from the linked list of the joints of the first body
        if (jointsList.joint == joint) {   // If the first element is the one to remove
            @Dimensionless
            JointListElement elementToRemove = jointsList;
            jointsList = elementToRemove.next;
        } else {    // If the element to remove is not the first one in the list
            @Dimensionless
            JointListElement currentElement = jointsList;
            while (currentElement.next != null) {
                if (currentElement.next.joint == joint) {
                    @Dimensionless
                    JointListElement elementToRemove = currentElement.next;
                    currentElement.next = elementToRemove.next;
                    break;
                }
                currentElement = currentElement.next;
            }
        }
    }

    // Return the angular velocity of the body
    public @Dimensionless Vector3 getAngularVelocity(@Dimensionless RigidBody this) {
        return angularVelocity;
    }

    public void setAngularVelocity(@Dimensionless RigidBody this, @Dimensionless Vector3 angularVelocity) {
        this.angularVelocity.set(angularVelocity);
    }

    // Return the linear velocity
    public @Dimensionless Vector3 getLinearVelocity(@Dimensionless RigidBody this) {
        return linearVelocity;
    }

    // Set the linear velocity of the rigid body
    public void setLinearVelocity(@Dimensionless RigidBody this, @Dimensionless Vector3 linearVelocity) {

        // If the body is able to move
        if (isMotionEnabled) {
            // Update the linear velocity of the current body state
            this.linearVelocity.set(linearVelocity);
        }
    }

    public @Dimensionless Vector3 getExternalForce(@Dimensionless RigidBody this) {
        return externalForce;
    }

    public @Dimensionless Vector3 getExternalTorque(@Dimensionless RigidBody this) {
        return externalTorque;
    }

    // Apply an external force to the body at a given point (in world-space coordinates).
    // If the point is not at the center of gravity of the body, it will also
    // generate some torque and therefore, change the angular velocity of the body.
    // If the body is sleeping, calling this method will wake it up. Note that the
    // force will we added to the sum of the applied forces and that this sum will be
    // reset to zero at the end of each call of the DynamicsWorld::update() method.
    public void applyForce(@Dimensionless RigidBody this, @Dimensionless Vector3 force, @Dimensionless Vector3 point) {

        // If it is a static body, do not apply any force
        if (!isMotionEnabled) {
            return;
        }

        // Awake the body if it was sleeping
        if (isSleeping) {
            setIsSleeping(false);
        }

        // Add the force and torque
        externalForce.add(force);
        externalTorque.add(new @Dimensionless Vector3(point).subtract(transform.getPosition()).cross(force));
    }

    // Apply an external force to the body at its gravity center.
    // If the body is sleeping, calling this method will wake it up. Note that the
    // force will we added to the sum of the applied forces and that this sum will be
    // reset to zero at the end of each call of the DynamicsWorld::update() method.
    public void applyForceToCenter(@Dimensionless RigidBody this, @Dimensionless Vector3 force) {

        // If it is a static body, do not apply any force
        if (!isMotionEnabled) {
            return;
        }

        // Awake the body if it was sleeping
        if (isSleeping) {
            setIsSleeping(false);
        }

        // Add the force
        externalForce.add(force);
    }

    // Apply an external torque to the body.
    // If the body is sleeping, calling this method will wake it up. Note that the
    // force will we added to the sum of the applied torques and that this sum will be
    // reset to zero at the end of each call of the DynamicsWorld::update() method.
    public void applyTorque(@Dimensionless RigidBody this, @Dimensionless Vector3 torque) {

        // If it is a static body, do not apply any force
        if (!isMotionEnabled) {
            return;
        }

        // Awake the body if it was sleeping
        if (isSleeping) {
            setIsSleeping(false);
        }

        // Add the torque
        externalTorque.add(torque);
    }

    // Return a reference to the material properties of the rigid body
    public @Dimensionless Material getMaterial(@Dimensionless RigidBody this) {
        return material;
    }

    // Set a new material for this rigid body
    public void setMaterial(@Dimensionless RigidBody this, @Dimensionless Material material) {
        this.material.set(material);
    }

    // Return the local inertia tensor of the body (in body coordinates)
    public @Dimensionless Matrix3x3 getInertiaTensorLocal(@Dimensionless RigidBody this) {
        return inertiaTensorLocal;
    }

    // Set the local inertia tensor of the body (in body coordinates)
    public void setInertiaTensorLocal(@Dimensionless RigidBody this, @Dimensionless Matrix3x3 inertiaTensorLocal) {

        // TODO: Set inertiaTensorLocalInverse when this is set?
        this.inertiaTensorLocal.set(inertiaTensorLocal);
    }

    // Return the inertia tensor in world coordinates.
    // The inertia tensor I_w in world coordinates is computed
    // with the local inertia tensor I_b in body coordinates
    // by I_w = R * I_b * R^T
    // where R is the rotation matrix (and R^T its transpose) of
    // the current orientation quaternion of the body
    public @Dimensionless Matrix3x3 getInertiaTensorWorld(@Dimensionless RigidBody this) {
        // TODO: Rename to new
        @Dimensionless
        Matrix3x3 rotation = transform.getOrientation().getMatrix(new @Dimensionless Matrix3x3());
        @Dimensionless
        Matrix3x3 transpose = new @Dimensionless Matrix3x3(rotation).transpose();

        // Compute and return the inertia tensor in world coordinates
        return new @Dimensionless Matrix3x3(rotation).multiply(inertiaTensorLocal).multiply(transpose);
    }

    // Get the inverse of the inertia tensor
    public @Dimensionless Matrix3x3 getInertiaTensorLocalInverse(@Dimensionless RigidBody this) {
        return inertiaTensorLocalInverse;
    }

    // Return the inverse of the inertia tensor in world coordinates.
    // The inertia tensor I_w in world coordinates is computed with the
    // local inverse inertia tensor I_b^-1 in body coordinates
    // by I_w = R * I_b^-1 * R^T
    // where R is the rotation matrix (and R^T its transpose) of the
    // current orientation quaternion of the body
    public @Dimensionless Matrix3x3 getInertiaTensorInverseWorld(@Dimensionless RigidBody this) {
        // TODO: Rename to new
        @Dimensionless
        Matrix3x3 rotation = transform.getOrientation().getMatrix(new @Dimensionless Matrix3x3());
        @Dimensionless
        Matrix3x3 transpose = new @Dimensionless Matrix3x3(rotation).transpose();

        // Compute and return the inertia tensor in world coordinates
        return new @Dimensionless Matrix3x3(rotation).multiply(inertiaTensorLocalInverse).multiply(transpose);
    }

    // Set the variable to know whether or not the body is sleeping
    @Override
    public void setIsSleeping(@Dimensionless RigidBody this, @Dimensionless boolean isSleeping) {
        if (isSleeping) {
            linearVelocity.zero();
            angularVelocity.zero();
            externalForce.zero();
            externalTorque.zero();
        }
        super.setIsSleeping(isSleeping);
    }

}
