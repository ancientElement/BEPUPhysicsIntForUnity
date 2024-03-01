﻿using System;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseEntries.MobileCollidables;
using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using BEPUutilities;
using FixMath.NET;

namespace BEPUphysics.NarrowPhaseSystems.Pairs
{
    ///<summary>
    /// Handles a mobile mesh-mobile mesh collision pair.
    ///</summary>
    public class MobileMeshTerrainPairHandler : MobileMeshMeshPairHandler
    {


        Terrain mesh;

        public override Collidable CollidableB
        {
            get { return mesh; }
        }
        public override Entities.Entity EntityB
        {
            get { return null; }
        }
        protected override Materials.Material MaterialB
        {
            get { return mesh.material; }
        }

        protected override TriangleCollidable GetOpposingCollidable(int index)
        {
            //Construct a TriangleCollidable from the static mesh.
            var toReturn = PhysicsResources.GetTriangleCollidable();
            Vector3 terrainUp = new Vector3(mesh.worldTransform.LinearTransform.M21, mesh.worldTransform.LinearTransform.M22, mesh.worldTransform.LinearTransform.M23);
            Fix64 dot;
            Vector3 AB, AC, normal;
            var shape = toReturn.Shape;
            mesh.Shape.GetTriangle(index, ref mesh.worldTransform, out shape.vA, out shape.vB, out shape.vC);
            Vector3 center;
            Vector3.Add(ref shape.vA, ref shape.vB, out center);
            Vector3.Add(ref center, ref shape.vC, out center);
            Vector3.Multiply(ref center, F64.OneThird, out center);
            Vector3.Subtract(ref shape.vA, ref center, out shape.vA);
            Vector3.Subtract(ref shape.vB, ref center, out shape.vB);
            Vector3.Subtract(ref shape.vC, ref center, out shape.vC);

            //The bounding box doesn't update by itself.
            toReturn.worldTransform.Position = center;
            toReturn.worldTransform.Orientation = Quaternion.Identity;
            toReturn.UpdateBoundingBoxInternal(F64.C0);

            Vector3.Subtract(ref shape.vB, ref shape.vA, out AB);
            Vector3.Subtract(ref shape.vC, ref shape.vA, out AC);
            Vector3.Cross(ref AB, ref AC, out normal);
            Vector3.Dot(ref terrainUp, ref normal, out dot);
            if (dot > F64.C0)
            {
                shape.sidedness = TriangleSidedness.Clockwise;
            }
            else
            {
                shape.sidedness = TriangleSidedness.Counterclockwise;
            }
            shape.collisionMargin = mobileMesh.Shape.MeshCollisionMargin;
            return toReturn;
        }


        protected override void ConfigureCollidable(TriangleEntry entry, Fix64 dt)
        {

        }

        ///<summary>
        /// Initializes the pair handler.
        ///</summary>
        ///<param name="entryA">First entry in the pair.</param>
        ///<param name="entryB">Second entry in the pair.</param>
        public override void Initialize(BroadPhaseEntry entryA, BroadPhaseEntry entryB)
        {
            mesh = entryA as Terrain;
            if (mesh == null)
            {
                mesh = entryB as Terrain;
                if (mesh == null)
                {
                    throw new ArgumentException("Inappropriate types used to initialize pair.");
                }
            }

            base.Initialize(entryA, entryB);
        }






        ///<summary>
        /// Cleans up the pair handler.
        ///</summary>
        public override void CleanUp()
        {

            base.CleanUp();
            mesh = null;


        }




        protected override void UpdateContainedPairs(Fix64 dt)
        {
            var overlappedElements = new QuickList<int>(BufferPools<int>.Thread);
            BoundingBox localBoundingBox;

            Vector3 sweep;
            Vector3.Multiply(ref mobileMesh.entity.linearVelocity, dt, out sweep);
            mobileMesh.Shape.GetSweptLocalBoundingBox(ref mobileMesh.worldTransform, ref mesh.worldTransform, ref sweep, out localBoundingBox);
            mesh.Shape.GetOverlaps(localBoundingBox, ref overlappedElements);
            for (int i = 0; i < overlappedElements.Count; i++)
            {
                TryToAdd(overlappedElements.Elements[i]);
            }

            overlappedElements.Dispose();

        }


    }
}
