// Auto-generated. Do not edit!

// (in-package robot_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ChassisMoveStatusRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.planner_state = null;
      this.speed_x_adjust = null;
      this.speed_y_adjust = null;
    }
    else {
      if (initObj.hasOwnProperty('planner_state')) {
        this.planner_state = initObj.planner_state
      }
      else {
        this.planner_state = 0;
      }
      if (initObj.hasOwnProperty('speed_x_adjust')) {
        this.speed_x_adjust = initObj.speed_x_adjust
      }
      else {
        this.speed_x_adjust = 0.0;
      }
      if (initObj.hasOwnProperty('speed_y_adjust')) {
        this.speed_y_adjust = initObj.speed_y_adjust
      }
      else {
        this.speed_y_adjust = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ChassisMoveStatusRequest
    // Serialize message field [planner_state]
    bufferOffset = _serializer.int32(obj.planner_state, buffer, bufferOffset);
    // Serialize message field [speed_x_adjust]
    bufferOffset = _serializer.float32(obj.speed_x_adjust, buffer, bufferOffset);
    // Serialize message field [speed_y_adjust]
    bufferOffset = _serializer.float32(obj.speed_y_adjust, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ChassisMoveStatusRequest
    let len;
    let data = new ChassisMoveStatusRequest(null);
    // Deserialize message field [planner_state]
    data.planner_state = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [speed_x_adjust]
    data.speed_x_adjust = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed_y_adjust]
    data.speed_y_adjust = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_msgs/ChassisMoveStatusRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0e6cb55e32b703b4709929c0381d7b9a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 客户端请求时设置状态
    int32 planner_state
    float32 speed_x_adjust
    float32 speed_y_adjust
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ChassisMoveStatusRequest(null);
    if (msg.planner_state !== undefined) {
      resolved.planner_state = msg.planner_state;
    }
    else {
      resolved.planner_state = 0
    }

    if (msg.speed_x_adjust !== undefined) {
      resolved.speed_x_adjust = msg.speed_x_adjust;
    }
    else {
      resolved.speed_x_adjust = 0.0
    }

    if (msg.speed_y_adjust !== undefined) {
      resolved.speed_y_adjust = msg.speed_y_adjust;
    }
    else {
      resolved.speed_y_adjust = 0.0
    }

    return resolved;
    }
};

class ChassisMoveStatusResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ChassisMoveStatusResponse
    // Serialize message field [result]
    bufferOffset = _serializer.int32(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ChassisMoveStatusResponse
    let len;
    let data = new ChassisMoveStatusResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robot_msgs/ChassisMoveStatusResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '034a8e20d6a306665e3a5b340fab3f09';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 result
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ChassisMoveStatusResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: ChassisMoveStatusRequest,
  Response: ChassisMoveStatusResponse,
  md5sum() { return '0a8ab4662f6938ca464a048217cc826e'; },
  datatype() { return 'robot_msgs/ChassisMoveStatus'; }
};
