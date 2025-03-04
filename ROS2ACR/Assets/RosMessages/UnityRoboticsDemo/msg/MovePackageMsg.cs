//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UnityRoboticsDemo
{
    [Serializable]
    public class MovePackageMsg : Message
    {
        public const string k_RosMessageName = "unity_robotics_demo_msgs/MovePackage";
        public override string RosMessageName => k_RosMessageName;

        public int package_id;
        public int source_location;
        public int target_location;
        public bool to_storage;

        public MovePackageMsg()
        {
            this.package_id = 0;
            this.source_location = 0;
            this.target_location = 0;
            this.to_storage = false;
        }

        public MovePackageMsg(int package_id, int source_location, int target_location, bool to_storage)
        {
            this.package_id = package_id;
            this.source_location = source_location;
            this.target_location = target_location;
            this.to_storage = to_storage;
        }

        public static MovePackageMsg Deserialize(MessageDeserializer deserializer) => new MovePackageMsg(deserializer);

        private MovePackageMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.package_id);
            deserializer.Read(out this.source_location);
            deserializer.Read(out this.target_location);
            deserializer.Read(out this.to_storage);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.package_id);
            serializer.Write(this.source_location);
            serializer.Write(this.target_location);
            serializer.Write(this.to_storage);
        }

        public override string ToString()
        {
            return "MovePackageMsg: " +
            "\npackage_id: " + package_id.ToString() +
            "\nsource_location: " + source_location.ToString() +
            "\ntarget_location: " + target_location.ToString() +
            "\nto_storage: " + to_storage.ToString();
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
