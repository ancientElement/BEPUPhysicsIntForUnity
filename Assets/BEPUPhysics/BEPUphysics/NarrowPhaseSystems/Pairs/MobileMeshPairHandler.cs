﻿using System;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.CollisionTests.CollisionAlgorithms.GJK;
using BEPUphysics.CollisionTests.Manifolds;
using BEPUphysics.Constraints.Collision;
using BEPUphysics.DataStructures;
using BEPUphysics.PositionUpdating;
using BEPUphysics.Settings;
 
using BEPUutilities;
using FixMath.NET;

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{
    ///<summary>
    /// Handles a mobile mesh-convex collision pair.
    ///</summary>
    public abstract class MobileMeshPairHandler : StandardPairHandler
    {
        MobileMeshCollidable mobileMesh;
        ConvexCollidable convex;

        private NonConvexContactManifoldConstraint contactConstraint;


        public override Collidable CollidableA
        {
            get { return convex; }
        }
        public override Collidable CollidableB
        {
            get { return mobileMesh; }
        }
        public override Entities.Entity EntityA
        {
            get { return convex.entity; }
        }
        public override Entities.Entity EntityB
        {
            get { return mobileMesh.entity; }
        }
        /// <summary>
        /// Gets the contact manifold used by the pair handler.
        /// </summary>
        public override ContactManifold ContactManifold
        {
            get { return MeshManifold; }
        }
        /// <summary>
        /// Gets the contact constraint used by the pair handler.
        /// </summary>
        public override ContactManifoldConstraint ContactConstraint
        {
            get { return contactConstraint; }
        }

        protected internal abstract MobileMeshContactManifold MeshManifold { get; }

        protected MobileMeshPairHandler()
        {
            contactConstraint = new NonConvexContactManifoldConstraint(this);
        }

        ///<summary>
        /// Initializes the pair handler.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        public override void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {

            mobileMesh = entryA as MobileMeshCollidable;
            convex = entryB as ConvexCollidable;

            if (mobileMesh == null || convex == null)
            {
                mobileMesh = entryB as MobileMeshCollidable;
                convex = entryA as ConvexCollidable;

                if (mobileMesh == null || convex == null)
                    throw new ArgumentException("Inappropriate types used to initialize pair.");
            }


            //Contact normal goes from A to B.
            broadPhaseOverlap.entryA = convex;
            broadPhaseOverlap.entryB = mobileMesh;

            //It's possible that the convex does not have an entity if it is a proxy for a non-entity collidable.
            //Similarly, the mesh could be a query object.
            UpdateMaterialProperties(convex.entity != null ? convex.entity.material : null, mobileMesh.entity != null ? mobileMesh.entity.material : null);


            base.Initialize(entryA, entryB);

        }


        ///<summary>
        /// Cleans up the pair handler.
        ///</summary>
        public override void CleanUp()
        {


            base.CleanUp();

            mobileMesh = null;
            convex = null;

        }



        ///<summary>
        /// Updates the time of impact for the pair.
        ///</summary>
        ///<param name="requester">Collidable requesting the update.</param>
        ///<param name="dt">Timestep duration.</param>
        public override void UpdateTimeOfImpact(Collidable requester, Fix64 dt)
        {
            var overlap = BroadPhaseOverlap;
            var meshMode = mobileMesh.entity == null ? PositionUpdateMode.Discrete : mobileMesh.entity.PositionUpdateMode;
            var convexMode = convex.entity == null ? PositionUpdateMode.Discrete : convex.entity.PositionUpdateMode;

            if (
                    (mobileMesh.IsActive || convex.IsActive) && //At least one has to be active.
                    (
                        (
                            convexMode == PositionUpdateMode.Continuous &&   //If both are continuous, only do the process for A.
                            meshMode == PositionUpdateMode.Continuous &&
                            overlap.entryA == requester
                        ) ||
                        (
                            convexMode == PositionUpdateMode.Continuous ^   //If only one is continuous, then we must do it.
                            meshMode == PositionUpdateMode.Continuous
                        )
                    )
                )
            {
                //TODO: This system could be made more robust by using a similar region-based rejection of edges.
                //CCD events are awfully rare under normal circumstances, so this isn't usually an issue.

                //Only perform the test if the minimum radii are small enough relative to the size of the velocity.
                Vector3 velocity;
                if (convexMode == PositionUpdateMode.Discrete)
                {                    
                    //Convex is static for the purposes of CCD.
                    Vector3.Negate(ref mobileMesh.entity.linearVelocity, out velocity);
                }
                else if (meshMode == PositionUpdateMode.Discrete)
                {
                    //Mesh is static for the purposes of CCD.
                    velocity = convex.entity.linearVelocity;
                }
                else
                {
                    //Both objects can move.
                    Vector3.Subtract(ref convex.entity.linearVelocity, ref mobileMesh.entity.linearVelocity, out velocity);

                }
                Vector3.Multiply(ref velocity, dt, out velocity);
                Fix64 velocitySquared = velocity.LengthSquared();

                var minimumRadius = convex.Shape.MinimumRadius * MotionSettings.CoreShapeScaling;
                timeOfImpact = F64.C1;
                if (minimumRadius * minimumRadius < velocitySquared)
                {
                    TriangleSidedness sidedness = mobileMesh.Shape.Sidedness;
                    Matrix3x3 orientation;
                    Matrix3x3.CreateFromQuaternion(ref mobileMesh.worldTransform.Orientation, out orientation);
                    var triangle = PhysicsThreadResources.GetTriangle();
                    triangle.collisionMargin = F64.C0;
                    //Spherecast against all triangles to find the earliest time.
                    for (int i = 0; i < MeshManifold.overlappedTriangles.Count; i++)
                    {
                        MeshBoundingBoxTreeData data = mobileMesh.Shape.TriangleMesh.Data;
                        int triangleIndex = MeshManifold.overlappedTriangles.Elements[i];
                        data.GetTriangle(triangleIndex, out triangle.vA, out triangle.vB, out triangle.vC);
                        Matrix3x3.Transform(ref triangle.vA, ref orientation, out triangle.vA);
                        Matrix3x3.Transform(ref triangle.vB, ref orientation, out triangle.vB);
                        Matrix3x3.Transform(ref triangle.vC, ref orientation, out triangle.vC);
                        Vector3.Add(ref triangle.vA, ref mobileMesh.worldTransform.Position, out triangle.vA);
                        Vector3.Add(ref triangle.vB, ref mobileMesh.worldTransform.Position, out triangle.vB);
                        Vector3.Add(ref triangle.vC, ref mobileMesh.worldTransform.Position, out triangle.vC);
                        //Put the triangle into 'localish' space of the convex.
                        Vector3.Subtract(ref triangle.vA, ref convex.worldTransform.Position, out triangle.vA);
                        Vector3.Subtract(ref triangle.vB, ref convex.worldTransform.Position, out triangle.vB);
                        Vector3.Subtract(ref triangle.vC, ref convex.worldTransform.Position, out triangle.vC);

                        RayHit rayHit;
                        if (GJKToolbox.CCDSphereCast(new Ray(Toolbox.ZeroVector, velocity), minimumRadius, triangle, ref Toolbox.RigidIdentity, timeOfImpact, out rayHit) &&
                            rayHit.T > Toolbox.BigEpsilon)
                        {

                            if (sidedness != TriangleSidedness.DoubleSided)
                            {
                                Vector3 AB, AC;
                                Vector3.Subtract(ref triangle.vB, ref triangle.vA, out AB);
                                Vector3.Subtract(ref triangle.vC, ref triangle.vA, out AC);
                                Vector3 normal;
                                Vector3.Cross(ref AB, ref AC, out normal);
                                Fix64 dot;
                                Vector3.Dot(ref normal, ref rayHit.Normal, out dot);
                                //Only perform sweep if the object is in danger of hitting the object.
                                //Triangles can be one sided, so check the impact normal against the triangle normal.
                                if (sidedness == TriangleSidedness.Counterclockwise && dot < F64.C0 ||
                                    sidedness == TriangleSidedness.Clockwise && dot > F64.C0)
                                {
                                    timeOfImpact = rayHit.T;
                                }
                            }
                            else
                            {
                                timeOfImpact = rayHit.T;
                            }
                        }
                    }
                    PhysicsThreadResources.GiveBack(triangle);
                }



            }

        }


        protected internal override void GetContactInformation(int index, out ContactInformation info)
        {
            info.Contact = MeshManifold.contacts.Elements[index];
            //Find the contact's normal and friction forces.
            info.FrictionImpulse = F64.C0;
            info.NormalImpulse = F64.C0;
            for (int i = 0; i < contactConstraint.frictionConstraints.Count; i++)
            {
                if (contactConstraint.frictionConstraints.Elements[i].PenetrationConstraint.contact == info.Contact)
                {
                    info.FrictionImpulse = contactConstraint.frictionConstraints.Elements[i].accumulatedImpulse;
                    info.NormalImpulse = contactConstraint.frictionConstraints.Elements[i].PenetrationConstraint.accumulatedImpulse;
                    break;
                }
            }

            //Compute relative velocity
            Vector3 velocity;
            if (convex.entity != null)
            {
                Vector3.Subtract(ref info.Contact.Position, ref convex.entity.position, out velocity);
                Vector3.Cross(ref convex.entity.angularVelocity, ref velocity, out velocity);
                Vector3.Add(ref velocity, ref convex.entity.linearVelocity, out info.RelativeVelocity);
            }
            else
                info.RelativeVelocity = new Vector3();

            if (mobileMesh.entity != null)
            {
                Vector3.Subtract(ref info.Contact.Position, ref mobileMesh.entity.position, out velocity);
                Vector3.Cross(ref mobileMesh.entity.angularVelocity, ref velocity, out velocity);
                Vector3.Add(ref velocity, ref mobileMesh.entity.linearVelocity, out velocity);
                Vector3.Subtract(ref info.RelativeVelocity, ref velocity, out info.RelativeVelocity);
            }

            info.Pair = this;
        }



    }

}
