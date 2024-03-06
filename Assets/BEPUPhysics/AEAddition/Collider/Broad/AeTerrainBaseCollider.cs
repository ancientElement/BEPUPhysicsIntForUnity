using AE_BEPUPhysics_Addition.Interface;
using BEPUutilities;
using FixMath.NET;
using UnityEngine;

namespace AE_BEPUPhysics_Addition
{
    public class AeTerrainBaseCollider : BaseCollider
    {
        [Range(2, 20)] public int resolutionScaleDiv = 10;
        [SerializeField] private TerrainData terrainData;
        private BEPUphysics.BroadPhaseEntries.Terrain m_terrain;

        public override void AddIntoSpace(BEPUphysics.Space space)
        {
            terrainData = GetComponent<Terrain>().terrainData;
            var resolusion = terrainData.heightmapResolution;
            var heights = terrainData.GetHeights(0, 0, resolusion, resolusion);
            var size = new UnityEngine.Vector3(terrainData.size.x / (resolusion / resolutionScaleDiv - 1), 1,
                terrainData.size.x / (resolusion / resolutionScaleDiv - 1)).ToFix64();

            var fixHeights = new Fix64[resolusion / resolutionScaleDiv, resolusion / resolutionScaleDiv];

            for (int i = 0; i < fixHeights.GetLength(0); i++)
            {
                for (int j = 0; j < fixHeights.GetLength(1); j++)
                {
                    fixHeights[i, j] = (heights[j * resolutionScaleDiv, i * resolutionScaleDiv] *
                                        terrainData.size.y).ToFix64();
                }
            }

            m_terrain = new BEPUphysics.BroadPhaseEntries.Terrain(fixHeights,
                new AffineTransform(size,
                    transform.rotation.ToFix64(),
                    transform.position.ToFix64()
                )
            );
            
            space.Add(m_terrain);
        }

        public override void RemoveFromSpace(BEPUphysics.Space space)
        {
            space.Remove(m_terrain);
        }
    }
}