﻿using System.Collections.Generic;
using BEPUutilities;
using BEPUutilities.ResourceManagement;
using FixMath.NET;

namespace BEPUphysics.DataStructures
{
    ///<summary>
    /// Data structure containing triangle mesh data and its associated bounding box tree.
    ///</summary>
    public class TriangleMesh
    {
        private MeshBoundingBoxTreeData data;
        ///<summary>
        /// Gets or sets the bounding box data used in the mesh.
        ///</summary>
        public MeshBoundingBoxTreeData Data
        {
            get
            {
                return data;
            }
            set
            {
                data = value;
                tree.Data = data;
            }
        }

        private MeshBoundingBoxTree tree;
        ///<summary>
        /// Gets the bounding box tree that accelerates queries to this triangle mesh.
        ///</summary>
        public MeshBoundingBoxTree Tree
        {
            get
            {
                return tree;
            }
        }

        ///<summary>
        /// Constructs a new triangle mesh.
        ///</summary>
        ///<param name="data">Data to use to construct the mesh.</param>
        public TriangleMesh(MeshBoundingBoxTreeData data)
        {
            this.data = data;
            tree = new MeshBoundingBoxTree(data);
        }

        ///<summary>
        /// Tests a ray against the triangle mesh.
        ///</summary>
        ///<param name="ray">Ray to test against the mesh.</param>
        ///<param name="hitCount">Number of hits between the ray and the mesh.</param>
        ///<returns>Whether or not the ray hit the mesh.</returns>
        public bool RayCast(Ray ray, out int hitCount)
        {
            var rayHits = CommonResources.GetRayHitList();
            bool toReturn = RayCast(ray, rayHits);
            hitCount = rayHits.Count;
            CommonResources.GiveBack(rayHits);
            return toReturn;
        }

        ///<summary>
        /// Tests a ray against the triangle mesh.
        ///</summary>
        ///<param name="ray">Ray to test against the mesh.</param>
        ///<param name="rayHit">Hit data for the ray, if any.</param>
        ///<returns>Whether or not the ray hit the mesh.</returns>
        public bool RayCast(Ray ray, out RayHit rayHit)
        {
            return RayCast(ray, Fix64.MaxValue, TriangleSidedness.DoubleSided, out rayHit);
        }

        ///<summary>
        /// Tests a ray against the triangle mesh.
        ///</summary>
        ///<param name="ray">Ray to test against the mesh.</param>
        /// <param name="sidedness">Sidedness to apply to the mesh for the ray cast.</param>
        ///<param name="rayHit">Hit data for the ray, if any.</param>
        ///<returns>Whether or not the ray hit the mesh.</returns>
        public bool RayCast(Ray ray, TriangleSidedness sidedness, out RayHit rayHit)
        {
            return RayCast(ray, Fix64.MaxValue, sidedness, out rayHit);
        }

        ///<summary>
        /// Tests a ray against the triangle mesh.
        ///</summary>
        ///<param name="ray">Ray to test against the mesh.</param>
        ///<param name="hits">Hit data for the ray, if any.</param>
        ///<returns>Whether or not the ray hit the mesh.</returns>
        public bool RayCast(Ray ray, IList<RayHit> hits)
        {
            return RayCast(ray, Fix64.MaxValue, TriangleSidedness.DoubleSided, hits);
        }

        ///<summary>
        /// Tests a ray against the triangle mesh.
        ///</summary>
        ///<param name="ray">Ray to test against the mesh.</param>
        /// <param name="sidedness">Sidedness to apply to the mesh for the ray cast.</param>
        ///<param name="hits">Hit data for the ray, if any.</param>
        ///<returns>Whether or not the ray hit the mesh.</returns>
        public bool RayCast(Ray ray, TriangleSidedness sidedness, IList<RayHit> hits)
        {
            return RayCast(ray, Fix64.MaxValue, sidedness, hits);
        }

        ///<summary>
        /// Tests a ray against the triangle mesh.
        ///</summary>
        ///<param name="ray">Ray to test against the mesh.</param>
        /// <param name="maximumLength">Maximum length of the ray in units of the ray direction's length.</param>
        ///<param name="rayHit">Hit data for the ray, if any.</param>
        ///<returns>Whether or not the ray hit the mesh.</returns>
        public bool RayCast(Ray ray, Fix64 maximumLength, out RayHit rayHit)
        {
            return RayCast(ray, maximumLength, TriangleSidedness.DoubleSided, out rayHit);
        }

        ///<summary>
        /// Tests a ray against the triangle mesh.
        ///</summary>
        ///<param name="ray">Ray to test against the mesh.</param>
        /// <param name="maximumLength">Maximum length of the ray in units of the ray direction's length.</param>
        /// <param name="sidedness">Sidedness to apply to the mesh for the ray cast.</param>
        ///<param name="rayHit">Hit data for the ray, if any.</param>
        ///<returns>Whether or not the ray hit the mesh.</returns>
        public bool RayCast(Ray ray, Fix64 maximumLength, TriangleSidedness sidedness, out RayHit rayHit)
        {
            var rayHits = CommonResources.GetRayHitList();
            bool toReturn = RayCast(ray, maximumLength, sidedness, rayHits);
            if (toReturn)
            {
                rayHit = rayHits[0];
                for (int i = 1; i < rayHits.Count; i++)
                {
                    RayHit hit = rayHits[i];
                    if (hit.T < rayHit.T)
                        rayHit = hit;
                }
            }
            else
                rayHit = new RayHit();
            CommonResources.GiveBack(rayHits);
            return toReturn;
        }

        ///<summary>
        /// Tests a ray against the triangle mesh.
        ///</summary>
        ///<param name="ray">Ray to test against the mesh.</param>
        /// <param name="maximumLength">Maximum length of the ray in units of the ray direction's length.</param>
        ///<param name="hits">Hit data for the ray, if any.</param>
        ///<returns>Whether or not the ray hit the mesh.</returns>
        public bool RayCast(Ray ray, Fix64 maximumLength, IList<RayHit> hits)
        {
            return RayCast(ray, maximumLength, TriangleSidedness.DoubleSided, hits);
        }

        ///<summary>
        /// Tests a ray against the triangle mesh.
        ///</summary>
        ///<param name="ray">Ray to test against the mesh.</param>
        /// <param name="maximumLength">Maximum length of the ray in units of the ray direction's length.</param>
        /// <param name="sidedness">Sidedness to apply to the mesh for the ray cast.</param>
        ///<param name="hits">Hit data for the ray, if any.</param>
        ///<returns>Whether or not the ray hit the mesh.</returns>
        public bool RayCast(Ray ray, Fix64 maximumLength, TriangleSidedness sidedness, IList<RayHit> hits)
        {
            var hitElements = CommonResources.GetIntList();
            tree.GetOverlaps(ray, maximumLength, hitElements);
            for (int i = 0; i < hitElements.Count; i++)
            {
                Vector3 v1, v2, v3;
                data.GetTriangle(hitElements[i], out v1, out v2, out v3);
                RayHit hit;
                if (Toolbox.FindRayTriangleIntersection(ref ray, maximumLength, sidedness, ref v1, ref v2, ref v3, out hit))
                {
                    hits.Add(hit);
                }
            }
            CommonResources.GiveBack(hitElements);
            return hits.Count > 0;
        }

        



    }
}
