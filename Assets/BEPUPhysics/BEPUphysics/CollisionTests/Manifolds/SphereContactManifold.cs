﻿using System;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUutilities.DataStructures;
using FixMath.NET;
using BEPUutilities;

namespace BEPUphysics.CollisionTests.Manifolds
{
    ///<summary>
    /// Manages persistent contact data between two boxes.
    ///</summary>
    public class SphereContactManifold : ContactManifold
    {
        protected ConvexCollidable<SphereShape> sphereA;
        protected ConvexCollidable<SphereShape> sphereB;

        ///<summary>
        /// Gets the first collidable in the pair.
        ///</summary>
        public ConvexCollidable<SphereShape> CollidableA
        {
            get
            {
                return sphereA;
            }
        }

        /// <summary>
        /// Gets the second collidable in the pair.
        /// </summary>
        public ConvexCollidable<SphereShape> CollidableB
        {
            get
            {
                return sphereB;
            }
        }

        ///<summary>
        /// Constructs a new manifold.
        ///</summary>
        public SphereContactManifold()
        {
            contacts = new RawList<Contact>(1);
        }

        Contact contact = new Contact();
        bool previouslyColliding;

        ///<summary>
        /// Updates the manifold.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public override void Update(Fix64 dt)
        {
            ContactData contactData;
            bool colliding = false;
            if (SphereTester.AreSpheresColliding(sphereA.Shape, sphereB.Shape, ref sphereA.worldTransform.Position, ref sphereB.worldTransform.Position, out contactData))
            {
                if (!previouslyColliding && contactData.PenetrationDepth >= F64.C0) //Don't use the contact if it's an initial contact and the depth is negative.  Why not? Bounciness and InitialCollisionDetected.
                {
                    Add(ref contactData);
                    colliding = true;
                }
                else if (previouslyColliding)
                {
                    contactData.Validate();
                    contact.Normal = contactData.Normal;
                    contact.PenetrationDepth = contactData.PenetrationDepth;
                    contact.Position = contactData.Position;
                    colliding = true;
                }
            }
            else
            {
                if (previouslyColliding)
                    Remove(0);
            }
            previouslyColliding = colliding;
        }

        protected override void Add(ref ContactData contactCandidate)
        {
            contactCandidate.Validate();
            contact.Normal = contactCandidate.Normal;
            contact.PenetrationDepth = contactCandidate.PenetrationDepth;
            contact.Position = contactCandidate.Position;

            contacts.Add(contact);
            OnAdded(contact);
        }

        protected override void Remove(int index)
        {
            contacts.RemoveAt(index);
            OnRemoved(contact);
        }



        ///<summary>
        /// Initializes the manifold.
        ///</summary>
        ///<param name="newCollidableA">First collidable.</param>
        ///<param name="newCollidableB">Second collidable.</param>
        ///<exception cref="Exception">Thrown when the collidables being used are not of the proper type.</exception>
        public override void Initialize(Collidable newCollidableA, Collidable newCollidableB)
        {
            sphereA = (ConvexCollidable<SphereShape>)newCollidableA;
            sphereB = (ConvexCollidable<SphereShape>)newCollidableB;

            if (sphereA == null || sphereB == null)
            {
                throw new ArgumentException("Inappropriate types used to initialize pair.");
            }

        }

        ///<summary>
        /// Cleans up the manifold.
        ///</summary>
        public override void CleanUp()
        {
            sphereA = null;
            sphereB = null;
            previouslyColliding = false;
            //We don't have to worry about losing a reference to our contact- we keep it local!
            contacts.Clear();
        }

        /// <summary>
        /// Clears the contacts associated with this manifold.
        /// </summary>
        public override void ClearContacts()
        {
            previouslyColliding = false;
            base.ClearContacts();
        }
    }
}
