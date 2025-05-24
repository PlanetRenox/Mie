using UnityEngine;

/// <summary>
/// Marker component that identifies objects as climbable ladders
/// </summary>
/// <remarks>
/// This is more efficient and type-safe than using tags or layers,
/// as it provides direct component access and clear code intent.
/// </remarks>
public class Ladder : MonoBehaviour {
    // This is an empty marker class
    // Used for identifying objects as ladders through GetComponentInParent<Ladder>()
}
