﻿using System;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.DataStructures;
using BEPUutilities;
using BEPUutilities.DataStructures;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUutilities.ResourceManagement;
using FixMath.NET;

namespace BEPUphysics.CollisionTests.Manifolds
{
    ///<summary>
    /// Manages persistent contacts between a convex and an instanced mesh.
    ///</summary>
    public abstract class MobileMeshContactManifold : TriangleMeshConvexContactManifold
    {
        protected MobileMeshCollidable mesh;
        internal int parentContactCount;

        internal RawList<int> overlappedTriangles = new RawList<int>(8);

        ///<summary>
        /// Gets the mesh of the pair.
        ///</summary>
        public MobileMeshCollidable Mesh
        {
            get
            {
                return mesh;
            }
        }

        protected override RigidTransform MeshTransform
        {
            get { return mesh.worldTransform; }
        }

        //Expand the convex's bounding box to include the mobile mesh's movement.

        protected internal override int FindOverlappingTriangles(Fix64 dt)
        {
            BoundingBox boundingBox;
            AffineTransform transform = new AffineTransform(mesh.worldTransform.Orientation, mesh.worldTransform.Position);
            convex.Shape.GetLocalBoundingBox(ref convex.worldTransform, ref transform, out boundingBox);
            Vector3 transformedVelocity;
            //Compute the relative velocity with respect to the mesh.  The mesh's bounding tree is NOT expanded with velocity,
            //so whatever motion there is between the two objects needs to be included in the convex's bounding box.

            if (convex.entity != null)
                transformedVelocity = convex.entity.linearVelocity;
            else
                transformedVelocity = new Vector3();
            if (mesh.entity != null)
                Vector3.Subtract(ref transformedVelocity, ref mesh.entity.linearVelocity, out transformedVelocity);

            //The linear transform is known to be orientation only, so using the transpose is allowed.
            Matrix3x3.TransformTranspose(ref transformedVelocity, ref transform.LinearTransform, out transformedVelocity);
            Vector3.Multiply(ref transformedVelocity, dt, out transformedVelocity);

            if (transformedVelocity.X > F64.C0)
                boundingBox.Max.X += transformedVelocity.X;
            else
                boundingBox.Min.X += transformedVelocity.X;

            if (transformedVelocity.Y > F64.C0)
                boundingBox.Max.Y += transformedVelocity.Y;
            else
                boundingBox.Min.Y += transformedVelocity.Y;

            if (transformedVelocity.Z > F64.C0)
                boundingBox.Max.Z += transformedVelocity.Z;
            else
                boundingBox.Min.Z += transformedVelocity.Z;

            mesh.Shape.TriangleMesh.Tree.GetOverlaps(boundingBox, overlappedTriangles);
            return overlappedTriangles.Count;
        }


        /// <summary>
        /// Precomputes the transform to bring triangles from their native local space to the local space of the convex.
        /// </summary>
        /// <param name="convexInverseWorldTransform">Inverse of the world transform of the convex shape.</param>
        /// <param name="fromMeshLocalToConvexLocal">Transform to apply to native local triangles to bring them into the local space of the convex.</param>
        protected override void PrecomputeTriangleTransform(ref AffineTransform convexInverseWorldTransform, out AffineTransform fromMeshLocalToConvexLocal)
        {
            //MobileMeshes only have TransformableMeshData sources.
            var data = ((TransformableMeshData)mesh.Shape.TriangleMesh.Data);
            //The mobile mesh has a shape-based transform followed by the rigid body transform.
            AffineTransform mobileMeshWorldTransform;
            AffineTransform.CreateFromRigidTransform(ref mesh.worldTransform, out mobileMeshWorldTransform);
            AffineTransform combinedMobileMeshWorldTransform;
            AffineTransform.Multiply(ref data.worldTransform, ref mobileMeshWorldTransform, out combinedMobileMeshWorldTransform);
            AffineTransform.Multiply(ref combinedMobileMeshWorldTransform, ref convexInverseWorldTransform, out fromMeshLocalToConvexLocal);
        }

        protected override bool ConfigureLocalTriangle(int i, TriangleShape localTriangleShape, out TriangleIndices indices)
        {
            var data = mesh.Shape.TriangleMesh.Data;
            int triangleIndex = overlappedTriangles.Elements[i];

            TriangleSidedness sidedness;
            //TODO: Note superhack; don't do this in v2.
            if (IsQuery)
                sidedness = TriangleSidedness.DoubleSided;
            else
            {
                switch (mesh.Shape.solidity)
                {
                    case MobileMeshSolidity.Clockwise:
                        sidedness = TriangleSidedness.Clockwise;
                        break;
                    case MobileMeshSolidity.Counterclockwise:
                        sidedness = TriangleSidedness.Counterclockwise;
                        break;
                    case MobileMeshSolidity.DoubleSided:
                        sidedness = TriangleSidedness.DoubleSided;
                        break;
                    default:
                        sidedness = mesh.Shape.SidednessWhenSolid;
                        break;
                }
            }
            localTriangleShape.sidedness = sidedness;
            localTriangleShape.collisionMargin = F64.C0;
            indices = new TriangleIndices
            {
                A = data.indices[triangleIndex],
                B = data.indices[triangleIndex + 1],
                C = data.indices[triangleIndex + 2]
            };

            localTriangleShape.vA = data.vertices[indices.A];
            localTriangleShape.vB = data.vertices[indices.B];
            localTriangleShape.vC = data.vertices[indices.C];

            return true;

        }

        protected internal override void CleanUpOverlappingTriangles()
        {
            overlappedTriangles.Clear();
        }

        protected override bool UseImprovedBoundaryHandling
        {
            get { return mesh.improveBoundaryBehavior; }
        }

        Fix64 previousDepth;
        Vector3 lastValidConvexPosition;
        protected override void ProcessCandidates(ref QuickList<ContactData> candidates)
        {
            if (candidates.Count == 0 && parentContactCount == 0 && Mesh.Shape.solidity == MobileMeshSolidity.Solid)
            {

                //If there's no new contacts on the mesh and it's supposed to be a solid,
                //then we must check the convex for containment within the shell.
                //We already know that it's not on the shell, meaning that the shape is either
                //far enough away outside the shell that there's no contact (and we're done), 
                //or it's far enough inside the shell that the triangles cannot create contacts.

                //To find out which it is, raycast against the shell.

                Matrix3x3 orientation;
                Matrix3x3.CreateFromQuaternion(ref mesh.worldTransform.Orientation, out orientation);

                Ray ray;
                Vector3.Subtract(ref convex.worldTransform.Position, ref mesh.worldTransform.Position, out ray.Position);
                Matrix3x3.TransformTranspose(ref ray.Position, ref orientation, out ray.Position);

                //Cast from the current position back to the previous position.
                Vector3.Subtract(ref lastValidConvexPosition, ref ray.Position, out ray.Direction);
                Fix64 rayDirectionLength = ray.Direction.LengthSquared();
                if (rayDirectionLength < Toolbox.Epsilon)
                {
                    //The object may not have moved enough to normalize properly.  If so, choose something arbitrary.
                    //Try the direction from the center of the object to the convex's position.
                    ray.Direction = ray.Position;
                    rayDirectionLength = ray.Direction.LengthSquared();
                    if (rayDirectionLength < Toolbox.Epsilon)
                    {
                        //This is unlikely; just pick something completely arbitrary then.
                        ray.Direction = Vector3.Up;
                        rayDirectionLength = F64.C1;
                    }
                }
                Vector3.Divide(ref ray.Direction, Fix64.Sqrt(rayDirectionLength), out ray.Direction);


                RayHit hit;
                if (mesh.Shape.IsLocalRayOriginInMesh(ref ray, out hit))
                {
                    ContactData newContact = new ContactData { Id = 2 };
                    //Give it a special id so that we know that it came from the inside.
                    Matrix3x3.Transform(ref ray.Position, ref orientation, out newContact.Position);
                    Vector3.Add(ref newContact.Position, ref mesh.worldTransform.Position, out newContact.Position);

                    newContact.Normal = hit.Normal;
                    newContact.Normal.Normalize();

                    Fix64 factor;
                    Vector3.Dot(ref ray.Direction, ref newContact.Normal, out factor);
                    newContact.PenetrationDepth = -factor * hit.T + convex.Shape.MinimumRadius;

                    Matrix3x3.Transform(ref newContact.Normal, ref orientation, out newContact.Normal);

                    newContact.Validate();

                    //Do not yet create a new contact.  Check to see if an 'inner contact' with id == 2 already exists.
                    bool addContact = true;
                    for (int i = 0; i < contacts.Count; i++)
                    {
                        if (contacts.Elements[i].Id == 2)
                        {
                            contacts.Elements[i].Position = newContact.Position;
                            contacts.Elements[i].Normal = newContact.Normal;
                            contacts.Elements[i].PenetrationDepth = newContact.PenetrationDepth;
                            supplementData.Elements[i].BasePenetrationDepth = newContact.PenetrationDepth;
                            supplementData.Elements[i].LocalOffsetA = new Vector3();
                            supplementData.Elements[i].LocalOffsetB = ray.Position; //convex local position in mesh.
                            addContact = false;
                            break;
                        }
                    }
                    if (addContact && contacts.Count == 0)
                        Add(ref newContact);
                    previousDepth = newContact.PenetrationDepth;
                }
                else
                {
                    //It's possible that we had a false negative.  The previous frame may have been in deep intersection, and this frame just failed to come to the same conclusion.
                    //If we set the target location to the current location, the object will never escape the mesh.  Instead, only do that if two frames agree that we are no longer colliding.
                    if (previousDepth > F64.C0)
                    {
                        //We're not touching the mesh.
                        lastValidConvexPosition = ray.Position;
                    }
                    previousDepth = F64.C0;

                }
            }
        }

        ///<summary>
        /// Cleans up the manifold.
        ///</summary>
        public override void CleanUp()
        {
            mesh = null;
            convex = null;
            parentContactCount = 0;
            base.CleanUp();
        }

        ///<summary>
        /// Initializes the manifold.
        ///</summary>
        ///<param name="newCollidableA">First collidable.</param>
        ///<param name="newCollidableB">Second collidable.</param>
        public override void Initialize(Collidable newCollidableA, Collidable newCollidableB)
        {
            convex = newCollidableA as ConvexCollidable;
            mesh = newCollidableB as MobileMeshCollidable;


            if (convex == null || mesh == null)
            {
                convex = newCollidableB as ConvexCollidable;
                mesh = newCollidableA as MobileMeshCollidable;
                if (convex == null || mesh == null)
                    throw new ArgumentException("Inappropriate types used to initialize contact manifold.");
            }

        }

        static LockingResourcePool<TriangleConvexPairTester> testerPool = new LockingResourcePool<TriangleConvexPairTester>();
        protected override void GiveBackTester(TrianglePairTester tester)
        {
            testerPool.GiveBack((TriangleConvexPairTester)tester);
        }

        protected override TrianglePairTester GetTester()
        {
            return testerPool.Take();
        }



    }
}
