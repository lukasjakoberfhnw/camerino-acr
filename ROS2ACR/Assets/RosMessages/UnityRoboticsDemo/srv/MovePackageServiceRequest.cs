//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UnityRoboticsDemo
{
    [Serializable]
    public class MovePackageServiceRequest : Message
    {
        public const string k_RosMessageName = "unity_robotics_demo_msgs/MovePackageService";
        public override string RosMessageName => k_RosMessageName;

        public MovePackageMsg input;

        public MovePackageServiceRequest()
        {
            this.input = new MovePackageMsg();
        }

        public MovePackageServiceRequest(MovePackageMsg input)
        {
            this.input = input;
        }

        public static MovePackageServiceRequest Deserialize(MessageDeserializer deserializer) => new MovePackageServiceRequest(deserializer);

        private MovePackageServiceRequest(MessageDeserializer deserializer)
        {
            this.input = MovePackageMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.input);
        }

        public override string ToString()
        {
            return "MovePackageServiceRequest: " +
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
