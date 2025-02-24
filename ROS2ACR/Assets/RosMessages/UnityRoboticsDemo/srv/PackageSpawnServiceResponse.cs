//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UnityRoboticsDemo
{
    [Serializable]
    public class PackageSpawnServiceResponse : Message
    {
        public const string k_RosMessageName = "unity_robotics_demo_msgs/PackageSpawnService";
        public override string RosMessageName => k_RosMessageName;

        public string status;
        public bool success;

        public PackageSpawnServiceResponse()
        {
            this.status = "";
            this.success = false;
        }

        public PackageSpawnServiceResponse(string status, bool success)
        {
            this.status = status;
            this.success = success;
        }

        public static PackageSpawnServiceResponse Deserialize(MessageDeserializer deserializer) => new PackageSpawnServiceResponse(deserializer);

        private PackageSpawnServiceResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.status);
            deserializer.Read(out this.success);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.status);
            serializer.Write(this.success);
        }

        public override string ToString()
        {
            return "PackageSpawnServiceResponse: " +
            "\nstatus: " + status.ToString() +
            "\nsuccess: " + success.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
