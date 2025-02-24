using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PackageSpawner : MonoBehaviour
{
    public GameObject packagePrefab; // Prefab for the package to spawn
    public Vector3 spawnAreaMin;    // Minimum bounds for spawn area
    public Vector3 spawnAreaMax;    // Maximum bounds for spawn area
    public float spawnInterval = 5f; // Interval in seconds for spawning packages
    public float packageCheckRadius = 1f; // Radius to check for existing packages

    private void Start()
    {
        // Start the spawning process
        StartCoroutine(SpawnPackagesRoutine());
    }

    private IEnumerator SpawnPackagesRoutine()
    {
        while (true)
        {
            // Wait for the spawn interval
            yield return new WaitForSeconds(spawnInterval);

            // Try to spawn a package
            TrySpawnPackage();
        }
    }

    private void TrySpawnPackage()
    {
        // Generate a random position within the spawn area
        Vector3 randomPosition = new Vector3(
            Random.Range(spawnAreaMin.x, spawnAreaMax.x),
            Random.Range(spawnAreaMin.y, spawnAreaMax.y),
            Random.Range(spawnAreaMin.z, spawnAreaMax.z)
        );

        // Check if the position is occupied by another package
        if (!IsPositionOccupied(randomPosition))
        {
            // Spawn the package at the position
            Instantiate(packagePrefab, randomPosition, Quaternion.identity);
        }
        else
        {
            Debug.Log("Position is occupied. Skipping spawn.");
        }
    }

    private bool IsPositionOccupied(Vector3 position)
    {
        // Check for overlapping colliders within a given radius
        Collider[] colliders = Physics.OverlapSphere(position, packageCheckRadius);

        // Return true if any colliders are found
        return colliders.Length > 0;
    }

    private void OnDrawGizmosSelected()
    {
        // Draw the spawn area for debugging
        Gizmos.color = Color.green;
        Gizmos.DrawWireCube((spawnAreaMin + spawnAreaMax) / 2, spawnAreaMax - spawnAreaMin);
    }
}
