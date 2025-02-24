//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UnityRoboticsDemo
{
    [Serializable]
    public class PackageSpawnServiceRequest : Message
    {
        public const string k_RosMessageName = "unity_robotics_demo_msgs/PackageSpawnService";
        public override string RosMessageName => k_RosMessageName;

        public PackageSpawnMsg input;

        public PackageSpawnServiceRequest()
        {
            this.input = new PackageSpawnMsg();
        }

        public PackageSpawnServiceRequest(PackageSpawnMsg input)
        {
            this.input = input;
        }

        public static PackageSpawnServiceRequest Deserialize(MessageDeserializer deserializer) => new PackageSpawnServiceRequest(deserializer);

        private PackageSpawnServiceRequest(MessageDeserializer deserializer)
        {
            this.input = PackageSpawnMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.input);
        }

        public override string ToString()
        {
            return "PackageSpawnServiceRequest: " +
            "\ninput: " + input.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
