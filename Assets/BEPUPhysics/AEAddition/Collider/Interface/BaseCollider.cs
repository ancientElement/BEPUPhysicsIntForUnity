using UnityEngine;

namespace AE_BEPUPhysics_Addition.Interface
{
    public abstract class BaseCollider : MonoBehaviour
    {
        public abstract void AddIntoSpace(BEPUphysics.Space space);
        public abstract void RemoveFromSpace(BEPUphysics.Space space);
    }
}