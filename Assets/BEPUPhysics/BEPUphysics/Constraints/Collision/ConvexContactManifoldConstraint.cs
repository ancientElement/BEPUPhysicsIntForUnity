﻿using BEPUphysics.CollisionTests;
using System.Collections.Generic;
using BEPUphysics.NarrowPhaseSystems.Pairs;
using BEPUutilities.DataStructures;
using FixMath.NET;
using BEPUutilities;

namespace BEPUphysics.Constraints.Collision
{
    ///<summary>
    /// Contact manifold constraint that is used by manifolds whose normals are assumed to be
    /// essentially the same.  This assumption can only be maintained between two convex objects.
    ///</summary>
    public class ConvexContactManifoldConstraint : ContactManifoldConstraint
    {
        //This contact manifold constraint covers a single, 4-contact pair.

        //The solver group is composed of multiple constraints.
        //One pentration constraint for each contact.
        //One sliding constraint.
        //One twist constraint.

        internal TwistFrictionConstraint twistFriction;
        ///<summary>
        /// Gets the twist friction constraint used by the manifold.
        ///</summary>
        public TwistFrictionConstraint TwistFriction
        {
            get
            {
                return twistFriction;
            }
        }
        internal SlidingFrictionTwoAxis slidingFriction;
        ///<summary>
        /// Gets the sliding friction constraint used by the manifold.
        ///</summary>
        public SlidingFrictionTwoAxis SlidingFriction
        {
            get
            {
                return slidingFriction;
            }
        }



        internal RawList<ContactPenetrationConstraint> penetrationConstraints;
        ///<summary>
        /// Gets the penetration constraints used by the manifold.
        ///</summary>
        public ReadOnlyList<ContactPenetrationConstraint> ContactPenetrationConstraints
        {
            get
            {
                return new ReadOnlyList<ContactPenetrationConstraint>(penetrationConstraints);
            }
        }

        Stack<ContactPenetrationConstraint> penetrationConstraintPool = new Stack<ContactPenetrationConstraint>(4);


        ///<summary>
        /// Constructs a new convex contact manifold constraint.
        ///</summary>
        public ConvexContactManifoldConstraint(CollidablePairHandler pairHandler)
            :base(pairHandler)
        {
            //All of the constraints are always in the solver group.  Some of them are just deactivated sometimes.
            //This reduces some bookkeeping complications.


            penetrationConstraints = new RawList<ContactPenetrationConstraint>(4);


            //Order matters in this adding process.  Sliding friction computes some information used by the twist friction, and both use penetration impulses.
            for (int i = 0; i < 4; i++)
            {
                var penetrationConstraint = new ContactPenetrationConstraint();
                Add(penetrationConstraint);
                penetrationConstraintPool.Push(penetrationConstraint);
            }
            slidingFriction = new SlidingFrictionTwoAxis();
            Add(slidingFriction);
            twistFriction = new TwistFrictionConstraint();
            Add(twistFriction);


        }

        ///<summary>
        /// Cleans up the constraint.
        ///</summary>
        public override void CleanUp()
        {
            base.CleanUp();
            //Deactivate any remaining constraints.
            for (int i = penetrationConstraints.Count - 1; i >= 0; i--)
            {
                var penetrationConstraint = penetrationConstraints.Elements[i];
                penetrationConstraint.CleanUp();
                penetrationConstraints.RemoveAt(i);
                penetrationConstraintPool.Push(penetrationConstraint);
            }
            if (twistFriction.isActive)
            {
                twistFriction.CleanUp();
                slidingFriction.CleanUp();
            }


        }


        ///<summary>
        /// Adds a contact to be managed by the constraint.
        ///</summary>
        ///<param name="contact">Contact to add.</param>
        public override void AddContact(Contact contact)
        {
            contact.Validate();
            var penetrationConstraint = penetrationConstraintPool.Pop();
            penetrationConstraint.Setup(this, contact);
            penetrationConstraints.Add(penetrationConstraint);
            if (!twistFriction.isActive)
            {
                //This is the first real contact.  All constraints need to become active.
                twistFriction.Setup(this);
                slidingFriction.Setup(this);
            }
        }

        ///<summary>
        /// Removes a contact from the constraint.
        ///</summary>
        ///<param name="contact">Contact to remove.</param>
        public override void RemoveContact(Contact contact)
        {
            for (int i = 0; i < penetrationConstraints.Count; i++)
            {
                ContactPenetrationConstraint penetrationConstraint;
                if ((penetrationConstraint = penetrationConstraints.Elements[i]).contact == contact)
                {
                    penetrationConstraint.CleanUp();
                    penetrationConstraints.RemoveAt(i);
                    penetrationConstraintPool.Push(penetrationConstraint);
                    break;
                }
            }
            if (penetrationConstraints.Count == 0)
            {
                //No more contacts.  Disable everything.
                //Don't have to worry about speculative contacts here; if there existed a regular manifold contact, there couldn't now exist a speculative contact.
                twistFriction.CleanUp();
                slidingFriction.CleanUp();
            }
        }



        //NOTE: Even though the order of addition to the solver group ensures penetration constraints come first, the
        //order of penetration constraints themselves matters in terms of determinism!
        //Consider what happens when penetration constraints are added and removed.  They cycle through a stack,
        //so the penetration constraints in the solver group's listing have inconsistent ordering.  Reloading the simulation
        //doesn't reset the penetration constraint pools, so suddenly everything is nonrepeatable, even single threaded.

        //By having the update use the order defined by contact addition/removal, determinism is maintained (so long as contact addition/removal is deterministic!)

        ///<summary>
        /// Performs the frame's configuration step.
        ///</summary>
        ///<param name="dt">Timestep duration.</param>
        public sealed override void Update(Fix64 dt)
        {
            for (int i = 0; i < penetrationConstraints.Count; i++)
                UpdateUpdateable(penetrationConstraints.Elements[i], dt);
            UpdateUpdateable(slidingFriction, dt);
            UpdateUpdateable(twistFriction, dt);
        }



        /// <summary>
        /// Performs any pre-solve iteration work that needs exclusive
        /// access to the members of the solver updateable.
        /// Usually, this is used for applying warmstarting impulses.
        /// </summary>
        public sealed override void ExclusiveUpdate()
        {
            for (int i = 0; i < penetrationConstraints.Count; i++)
                ExclusiveUpdateUpdateable(penetrationConstraints.Elements[i]);
            ExclusiveUpdateUpdateable(slidingFriction);
            ExclusiveUpdateUpdateable(twistFriction);
        }


        /// <summary>
        /// Computes one iteration of the constraint to meet the solver updateable's goal.
        /// </summary>
        /// <returns>The rough applied impulse magnitude.</returns>
        public sealed override Fix64 SolveIteration()
        {

            int activeConstraints = 0;
            for (int i = 0; i < penetrationConstraints.Count; i++)
                SolveUpdateable(penetrationConstraints.Elements[i], ref activeConstraints);
            SolveUpdateable(slidingFriction, ref activeConstraints);
            SolveUpdateable(twistFriction, ref activeConstraints);


            isActiveInSolver = activeConstraints > 0;

            return solverSettings.minimumImpulse + F64.C1; //Never let the system deactivate due to low impulses; solver group takes care of itself.
        }

    }
}
