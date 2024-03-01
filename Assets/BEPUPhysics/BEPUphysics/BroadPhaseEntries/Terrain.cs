﻿using System;
using BEPUphysics.BroadPhaseEntries.Events;
using BEPUutilities;
using BEPUphysics.CollisionShapes;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUphysics.OtherSpaceStages;
using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using FixMath.NET;

namespace BEPUphysics.BroadPhaseEntries
{
    ///<summary>
    /// Heightfield-based unmovable collidable object.
    ///</summary>
    public class Terrain : StaticCollidable
    {
        ///<summary>
        /// Gets the shape of this collidable.
        ///</summary>
        public new TerrainShape Shape
        {
            get
            {
                return (TerrainShape)shape;
            }
            set
            {
                base.Shape = value;
            }
        }

        /// <summary>
        /// The sidedness of triangles in the terrain. Precomputed based on the transform.
        /// </summary>
        internal TriangleSidedness sidedness;

        internal AffineTransform worldTransform;
        ///<summary>
        /// Gets or sets the affine transform of the terrain.
        ///</summary>
        public AffineTransform WorldTransform
        {
            get
            {
                return worldTransform;
            }
            set
            {
                worldTransform = value;

                //Sidedness must be calibrated based on the transform.
                //To do this, note a few things:
                //1) All triangles have the same sidedness in the terrain. Winding is consistent. Calibrating for one triangle calibrates for all.
                //2) Taking a triangle from the terrain into world space and computing the normal there for comparison is unneeded. Picking a fixed valid normal in local space (like {0, 1, 0}) is sufficient.
                //3) Normals can't be transformed by a direct application of a general affine transform. The adjugate transpose must be used.

                Matrix3x3 normalTransform;
                Matrix3x3.AdjugateTranspose(ref worldTransform.LinearTransform, out normalTransform);

                //If the world 'up' doesn't match the normal 'up', some reflection occurred which requires a winding flip.
                if (Vector3.Dot(normalTransform.Up, worldTransform.LinearTransform.Up) < F64.C0)
                {
                    sidedness = TriangleSidedness.Clockwise;
                }
                else
                {
                    sidedness = TriangleSidedness.Counterclockwise;
                }


            }
        }


        internal bool improveBoundaryBehavior = true;
        /// <summary>
        /// Gets or sets whether or not the collision system should attempt to improve contact behavior at the boundaries between triangles.
        /// This has a slight performance cost, but prevents objects sliding across a triangle boundary from 'bumping,' and otherwise improves
        /// the robustness of contacts at edges and vertices.
        /// </summary>
        public bool ImproveBoundaryBehavior
        {
            get
            {
                return improveBoundaryBehavior;
            }
            set
            {
                improveBoundaryBehavior = value;
            }
        }

        protected internal ContactEventManager<Terrain> events;
        ///<summary>
        /// Gets the event manager used by the Terrain.
        ///</summary>
        public ContactEventManager<Terrain> Events
        {
            get
            {
                return events;
            }
            set
            {
                if (value.Owner != null && //Can't use a manager which is owned by a different entity.
                    value != events) //Stay quiet if for some reason the same event manager is being set.
                    throw new ArgumentException("Event manager is already owned by a Terrain; event managers cannot be shared.");
                if (events != null)
                    events.Owner = null;
                events = value;
                if (events != null)
                    events.Owner = this;
            }
        }

        protected internal override IContactEventTriggerer EventTriggerer
        {
            get { return events; }
        }

        protected override IDeferredEventCreator EventCreator
        {
            get { return events; }
        }


        internal Fix64 thickness;
        /// <summary>
        /// Gets or sets the thickness of the terrain.  This defines how far below the triangles of the terrain's surface the terrain 'body' extends.
        /// Anything within the body of the terrain will be pulled back up to the surface.
        /// </summary>
        public Fix64 Thickness
        {
            get
            {
                return thickness;
            }
            set
            {
                if (value < F64.C0)
                    throw new ArgumentException("Cannot use a negative thickness value.");

                //Modify the bounding box to include the new thickness.
                Vector3 down = Vector3.Normalize(worldTransform.LinearTransform.Down);
                Vector3 thicknessOffset = down * (value - thickness);
                //Use the down direction rather than the thicknessOffset to determine which
                //component of the bounding box to subtract, since the down direction contains all
                //previous extra thickness.
                if (down.X < F64.C0)
                    boundingBox.Min.X += thicknessOffset.X;
                else
                    boundingBox.Max.X += thicknessOffset.X;
                if (down.Y < F64.C0)
                    boundingBox.Min.Y += thicknessOffset.Y;
                else
                    boundingBox.Max.Y += thicknessOffset.Y;
                if (down.Z < F64.C0)
                    boundingBox.Min.Z += thicknessOffset.Z;
                else
                    boundingBox.Max.Z += thicknessOffset.Z;

                thickness = value;
            }
        }


        ///<summary>
        /// Constructs a new Terrain.
        ///</summary>
        ///<param name="shape">Shape to use for the terrain.</param>
        ///<param name="worldTransform">Transform to use for the terrain.</param>
        public Terrain(TerrainShape shape, AffineTransform worldTransform)
        {
            WorldTransform = worldTransform;
            Shape = shape;

            Events = new ContactEventManager<Terrain>();
        }


        ///<summary>
        /// Constructs a new Terrain.
        ///</summary>
        ///<param name="heights">Height data to use to create the TerrainShape.</param>
        ///<param name="worldTransform">Transform to use for the terrain.</param>
        public Terrain(Fix64[,] heights, AffineTransform worldTransform)
            : this(new TerrainShape(heights), worldTransform)
        {
        }


        ///<summary>
        /// Updates the bounding box of the terrain.
        ///</summary>
        public override void UpdateBoundingBox()
        {
            Shape.GetBoundingBox(ref worldTransform, out boundingBox);
            //Include the thickness of the terrain.
            Vector3 thicknessOffset = Vector3.Normalize(worldTransform.LinearTransform.Down) * thickness;
            if (thicknessOffset.X < F64.C0)
                boundingBox.Min.X += thicknessOffset.X;
            else
                boundingBox.Max.X += thicknessOffset.X;
            if (thicknessOffset.Y < F64.C0)
                boundingBox.Min.Y += thicknessOffset.Y;
            else
                boundingBox.Max.Y += thicknessOffset.Y;
            if (thicknessOffset.Z < F64.C0)
                boundingBox.Min.Z += thicknessOffset.Z;
            else
                boundingBox.Max.Z += thicknessOffset.Z;
        }



        /// <summary>
        /// Tests a ray against the entry.
        /// </summary>
        /// <param name="ray">Ray to test.</param>
        /// <param name="maximumLength">Maximum length, in units of the ray's direction's length, to test.</param>
        /// <param name="rayHit">Hit location of the ray on the entry, if any.</param>
        /// <returns>Whether or not the ray hit the entry.</returns>
        public override bool RayCast(Ray ray, Fix64 maximumLength, out RayHit rayHit)
        {
            return Shape.RayCast(ref ray, maximumLength, ref worldTransform, out rayHit);
        }

        /// <summary>
        /// Casts a convex shape against the collidable.
        /// </summary>
        /// <param name="castShape">Shape to cast.</param>
        /// <param name="startingTransform">Initial transform of the shape.</param>
        /// <param name="sweep">Sweep to apply to the shape.</param>
        /// <param name="hit">Hit data, if any.</param>
        /// <returns>Whether or not the cast hit anything.</returns>
        public override bool ConvexCast(CollisionShapes.ConvexShapes.ConvexShape castShape, ref RigidTransform startingTransform, ref Vector3 sweep, out RayHit hit)
        {
            hit = new RayHit();
            BoundingBox localSpaceBoundingBox;
            castShape.GetSweptLocalBoundingBox(ref startingTransform, ref worldTransform, ref sweep, out localSpaceBoundingBox);
            var tri = PhysicsThreadResources.GetTriangle();
            var hitElements = new QuickList<int>(BufferPools<int>.Thread);
            if (Shape.GetOverlaps(localSpaceBoundingBox, ref hitElements))
            {
                hit.T = Fix64.MaxValue;
                for (int i = 0; i < hitElements.Count; i++)
                {
                    Shape.GetTriangle(hitElements.Elements[i], ref worldTransform, out tri.vA, out tri.vB, out tri.vC);
                    Vector3 center;
                    Vector3.Add(ref tri.vA, ref tri.vB, out center);
                    Vector3.Add(ref center, ref tri.vC, out center);
                    Vector3.Multiply(ref center, F64.OneThird, out center);
                    Vector3.Subtract(ref tri.vA, ref center, out tri.vA);
                    Vector3.Subtract(ref tri.vB, ref center, out tri.vB);
                    Vector3.Subtract(ref tri.vC, ref center, out tri.vC);
                    tri.MaximumRadius = tri.vA.LengthSquared();
                    Fix64 radius = tri.vB.LengthSquared();
                    if (tri.MaximumRadius < radius)
                        tri.MaximumRadius = radius;
                    radius = tri.vC.LengthSquared();
                    if (tri.MaximumRadius < radius)
                        tri.MaximumRadius = radius;
                    tri.MaximumRadius = Fix64.Sqrt(tri.MaximumRadius);
                    tri.collisionMargin = F64.C0;
                    var triangleTransform = new RigidTransform { Orientation = Quaternion.Identity, Position = center };
                    RayHit tempHit;
                    if (MPRToolbox.Sweep(castShape, tri, ref sweep, ref Toolbox.ZeroVector, ref startingTransform, ref triangleTransform, out tempHit) && tempHit.T < hit.T)
                    {
                        hit = tempHit;
                    }
                }
                tri.MaximumRadius = F64.C0;
                PhysicsThreadResources.GiveBack(tri);
                hitElements.Dispose();
                return hit.T != Fix64.MaxValue;
            }
            PhysicsThreadResources.GiveBack(tri);
            hitElements.Dispose();
            return false;
        }

        ///<summary>
        /// Gets the position of a vertex at the given indices.
        ///</summary>
        ///<param name="i">First dimension index into the heightmap array.</param>
        ///<param name="j">Second dimension index into the heightmap array.</param>
        ///<param name="position">Position at the given indices.</param>
        public void GetPosition(int i, int j, out Vector3 position)
        {
            Shape.GetPosition(i, j, ref worldTransform, out position);
        }





    }
}
