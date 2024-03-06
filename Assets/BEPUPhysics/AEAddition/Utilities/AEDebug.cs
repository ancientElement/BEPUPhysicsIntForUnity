#define DEBUGMODE

namespace AE_BEPUPhysics_Addition
{
    public static class AEDebug
    {
        public static void Log(string message)
        {
#if DEBUGMODE
            UnityEngine.Debug.Log(message);
#endif
        }

        public static void Log(string message, UnityEngine.Object obj)
        {
#if DEBUGMODE
             UnityEngine.Debug.Log(message, obj);
#endif
        }
    }
}