using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Detects if the camera is underwater by tracking water trigger colliders
/// </summary>
public class CameraWaterCheck : MonoBehaviour {
    
    // Use HashSet for more efficient contains/add/remove operations
    private readonly HashSet<Collider> triggers = new HashSet<Collider>();
    
    /// <summary>
    /// Track colliders when entering a trigger
    /// </summary>
    private void OnTriggerEnter(Collider other) {
        // Only add if not already in the set (HashSet handles this implicitly)
        triggers.Add(other);
    }

    /// <summary>
    /// Remove colliders when exiting a trigger
    /// </summary>
    private void OnTriggerExit(Collider other) {
        // Remove from the set (no need to check contains first)
        triggers.Remove(other);
    }

    /// <summary>
    /// Check if the camera is underwater
    /// </summary>
    /// <returns>True if underwater, false otherwise</returns>
    public bool IsUnderwater() {
        // Early exit if no triggers
        if (triggers.Count == 0) {
            return false;
        }
        
        // Check if any trigger is a water collider
        foreach (Collider trigger in triggers) {
            // Skip null triggers (could happen if objects are destroyed)
            if (trigger == null) {
                continue;
            }
            
            // Check if this trigger belongs to a Water component
            if (trigger.GetComponentInParent<Water>() != null) {
                return true;
            }
        }
        
        return false;
    }
}
