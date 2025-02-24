using UnityEngine;

public class PackageBehavior : MonoBehaviour
{
    public enum PackageType { ToBeWarehoused, Delivered }
    public PackageType packageType; // Identify if the package is to be warehoused or delivered

    private float deliveryTime; // Time when the package was delivered

    private void Start()
    {
        if (packageType == PackageType.Delivered)
        {
            // Record the time of delivery
            deliveryTime = Time.time;
        }
    }

    private void Update()
    {
        if (packageType == PackageType.Delivered)
        {
            // Destroy the package if it has been at the station for 2 seconds
            if (Time.time - deliveryTime >= 2f)
            {
                Destroy(gameObject);
            }
        }
    }
}
