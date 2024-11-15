// Auto-generated. Do not edit!

// (in-package robot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Track_reset {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.Track_id = null;
      this.Reset = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('Track_id')) {
        this.Track_id = initObj.Track_id
      }
      else {
        this.Track_id = 0;
      }
      if (initObj.hasOwnProperty('Reset')) {
        this.Reset = initObj.Reset
      }
      else {
        this.Reset = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Track_reset
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [Track_id]
    bufferOffset = _serializer.int8(obj.Track_id, buffer, bufferOffset);
    // Serialize message field [Reset]
    bufferOffset = _serializer.int8(obj.Reset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Track_reset
    let len;
    let data = new Track_reset(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [Track_id]
    data.Track_id = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [Reset]
    data.Reset = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_msgs/Track_reset';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c12ffa81e03dadac12770fb057786612';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int8 Track_id
    int8 Reset
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Track_reset(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.Track_id !== undefined) {
      resolved.Track_id = msg.Track_id;
    }
    else {
      resolved.Track_id = 0
    }

    if (msg.Reset !== undefined) {
      resolved.Reset = msg.Reset;
    }
    else {
      resolved.Reset = 0
    }

    return resolved;
    }
};

module.exports = Track_reset;
