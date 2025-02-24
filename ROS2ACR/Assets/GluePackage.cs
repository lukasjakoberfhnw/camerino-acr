using UnityEngine;

public class GluePackage : MonoBehaviour
{
    private bool isAttached = false;
    private GameObject attachedPackage;
    void FixedUpdate()
    {
        if(isAttached)
        {
            // check if GameObject still exists
            if (attachedPackage == null)
            {
                Debug.LogWarning("Package not found.");
                isAttached = false;
                return;
            }
        }

        // If the package is attached to the fork, check for input to release it
        if (isAttached && Input.GetKey(KeyCode.E))
        {
            Debug.Log("PRESSIGN E");
            // Get the Fixed Joint component

            // check if GameObject still exists
            if (attachedPackage == null)
            {
                Debug.LogWarning("Package not found.");
                isAttached = false;
                return;
            }

            FixedJoint joint = attachedPackage.GetComponent<FixedJoint>();
            // Ensure the joint exists
            if (joint == null)
            {
                Debug.LogWarning("Fixed Joint not found on the package.");
                return;
            }

            // Destroy the Fixed Joint
            Destroy(joint);

            Debug.Log("Package detached from the fork.");
            isAttached = false;
        }

        
        // if package is near an object with tag dropoff by matching collider, detach it
        Collider[] colliders = Physics.OverlapSphere(transform.position, 0.5f);
        foreach (Collider collider in colliders)
        {
            if (collider.gameObject.CompareTag("Dropoff")) //  && isAttached
            {
                Debug.LogWarning("Package delivered to the shelf.");
                
                // Get the Fixed Joint component
                FixedJoint joint = attachedPackage.GetComponent<FixedJoint>();
                // Ensure the joint exists
                if (joint == null)
                {
                    Debug.LogWarning("Fixed Joint not found on the package.");
                    return;
                }

                // Destroy the Fixed Joint
                Destroy(joint);
                isAttached = false;
            }
        }
    }
    void OnCollisionEnter(Collision collision)
    {
        // Check if the object colliding with the fork has the "Pickup" tag
        if (collision.gameObject.CompareTag("Package") && !isAttached)
        {
            // Get the Rigidbody of the object
            Rigidbody objectRb = collision.gameObject.GetComponent<Rigidbody>();

            // Ensure the object has a Rigidbody
            if (objectRb != null)
            {
                // Add a Fixed Joint to the object
                FixedJoint joint = collision.gameObject.AddComponent<FixedJoint>();

                // Attach the joint to the fork's Rigidbody
                joint.connectedBody = GetComponent<Rigidbody>();

                attachedPackage = collision.gameObject;

                Debug.Log("Package glued to the fork.");
                isAttached = true;
            }
        }
        if(collision.gameObject.CompareTag("Dropoff")) //  && isAttached
        {
            Debug.Log("Package delivered to the shelf.");
            
            // Get the Fixed Joint component
            FixedJoint joint = attachedPackage.GetComponent<FixedJoint>();
            // Ensure the joint exists
            if (joint == null)
            {
                Debug.LogWarning("Fixed Joint not found on the package.");
                return;
            }

            // Destroy the Fixed Joint
            Destroy(joint);
            isAttached = false;
        }
    }
}