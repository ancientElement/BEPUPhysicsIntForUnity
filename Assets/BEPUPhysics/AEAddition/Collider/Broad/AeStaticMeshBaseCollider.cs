using System;
using System.Linq;
using AE_BEPUPhysics_Addition.Interface;
using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.Entities;
using BEPUutilities;
using UnityEngine;
using Object = UnityEngine.Object;
using Space = BEPUphysics.Space;

namespace AE_BEPUPhysics_Addition
{
    public class AeStaticMeshBaseCollider : BaseCollider
    {
        [SerializeField] private Mesh m_mesh;
        private StaticMesh m_staticMesh;

        public override void AddIntoSpace(Space space)
        {
            m_mesh = Object.Instantiate(GetComponent<MeshFilter>().mesh);
            var affineTransform = new AffineTransform(transform.lossyScale.ToFix64(), transform.rotation.ToFix64(),
                transform.position.ToFix64());
            m_staticMesh = new StaticMesh(m_mesh.vertices.ToFix64().ToArray(),m_mesh.triangles,affineTransform);
            space.Add(m_staticMesh);
        }

        public override void RemoveFromSpace(Space space)
        {
            space.Remove(m_staticMesh);
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            Gizmos.color = Color.blue;
            Matrix4x4 newLocalToWorldMatrix = transform.localToWorldMatrix;
            Gizmos.matrix = newLocalToWorldMatrix;
            Gizmos.DrawWireMesh(GetComponent<MeshFilter>().sharedMesh);
        }
#endif
    }
}