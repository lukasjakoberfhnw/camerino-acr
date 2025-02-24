using UnityEngine;

public class MoveForward : MonoBehaviour
{
    public float speed = 5f; // Speed of movement
    private Rigidbody rb;    // Reference to the Rigidbody component
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        // Move the Rigidbody forward based on its local forward direction
        Vector3 forwardMovement = transform.forward * speed;
        rb.MovePosition(rb.position - forwardMovement * Time.fixedDeltaTime);
    }
}
