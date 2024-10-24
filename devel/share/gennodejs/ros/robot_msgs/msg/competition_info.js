// Auto-generated. Do not edit!

// (in-package robot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class competition_info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.game_state = null;
      this.our_outpost_hp = null;
      this.enemy_outpost_hp = null;
      this.remain_bullet = null;
      this.enemy_sentry_hp = null;
      this.our_sentry_hp = null;
      this.our_base_hp = null;
      this.first_blood = null;
      this.target_position_x = null;
      this.target_position_y = null;
      this.is_target_active = null;
    }
    else {
      if (initObj.hasOwnProperty('game_state')) {
        this.game_state = initObj.game_state
      }
      else {
        this.game_state = 0;
      }
      if (initObj.hasOwnProperty('our_outpost_hp')) {
        this.our_outpost_hp = initObj.our_outpost_hp
      }
      else {
        this.our_outpost_hp = 0;
      }
      if (initObj.hasOwnProperty('enemy_outpost_hp')) {
        this.enemy_outpost_hp = initObj.enemy_outpost_hp
      }
      else {
        this.enemy_outpost_hp = 0;
      }
      if (initObj.hasOwnProperty('remain_bullet')) {
        this.remain_bullet = initObj.remain_bullet
      }
      else {
        this.remain_bullet = 0;
      }
      if (initObj.hasOwnProperty('enemy_sentry_hp')) {
        this.enemy_sentry_hp = initObj.enemy_sentry_hp
      }
      else {
        this.enemy_sentry_hp = 0;
      }
      if (initObj.hasOwnProperty('our_sentry_hp')) {
        this.our_sentry_hp = initObj.our_sentry_hp
      }
      else {
        this.our_sentry_hp = 0;
      }
      if (initObj.hasOwnProperty('our_base_hp')) {
        this.our_base_hp = initObj.our_base_hp
      }
      else {
        this.our_base_hp = 0;
      }
      if (initObj.hasOwnProperty('first_blood')) {
        this.first_blood = initObj.first_blood
      }
      else {
        this.first_blood = 0;
      }
      if (initObj.hasOwnProperty('target_position_x')) {
        this.target_position_x = initObj.target_position_x
      }
      else {
        this.target_position_x = 0.0;
      }
      if (initObj.hasOwnProperty('target_position_y')) {
        this.target_position_y = initObj.target_position_y
      }
      else {
        this.target_position_y = 0.0;
      }
      if (initObj.hasOwnProperty('is_target_active')) {
        this.is_target_active = initObj.is_target_active
      }
      else {
        this.is_target_active = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type competition_info
    // Serialize message field [game_state]
    bufferOffset = _serializer.uint8(obj.game_state, buffer, bufferOffset);
    // Serialize message field [our_outpost_hp]
    bufferOffset = _serializer.uint16(obj.our_outpost_hp, buffer, bufferOffset);
    // Serialize message field [enemy_outpost_hp]
    bufferOffset = _serializer.uint16(obj.enemy_outpost_hp, buffer, bufferOffset);
    // Serialize message field [remain_bullet]
    bufferOffset = _serializer.uint16(obj.remain_bullet, buffer, bufferOffset);
    // Serialize message field [enemy_sentry_hp]
    bufferOffset = _serializer.uint16(obj.enemy_sentry_hp, buffer, bufferOffset);
    // Serialize message field [our_sentry_hp]
    bufferOffset = _serializer.uint16(obj.our_sentry_hp, buffer, bufferOffset);
    // Serialize message field [our_base_hp]
    bufferOffset = _serializer.uint16(obj.our_base_hp, buffer, bufferOffset);
    // Serialize message field [first_blood]
    bufferOffset = _serializer.uint8(obj.first_blood, buffer, bufferOffset);
    // Serialize message field [target_position_x]
    bufferOffset = _serializer.float32(obj.target_position_x, buffer, bufferOffset);
    // Serialize message field [target_position_y]
    bufferOffset = _serializer.float32(obj.target_position_y, buffer, bufferOffset);
    // Serialize message field [is_target_active]
    bufferOffset = _serializer.int8(obj.is_target_active, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type competition_info
    let len;
    let data = new competition_info(null);
    // Deserialize message field [game_state]
    data.game_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [our_outpost_hp]
    data.our_outpost_hp = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [enemy_outpost_hp]
    data.enemy_outpost_hp = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [remain_bullet]
    data.remain_bullet = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [enemy_sentry_hp]
    data.enemy_sentry_hp = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [our_sentry_hp]
    data.our_sentry_hp = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [our_base_hp]
    data.our_base_hp = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [first_blood]
    data.first_blood = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [target_position_x]
    data.target_position_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [target_position_y]
    data.target_position_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [is_target_active]
    data.is_target_active = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 23;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_msgs/competition_info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0126e094b04a5313899b587ef4b04645';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 game_state
    uint16 our_outpost_hp
    uint16 enemy_outpost_hp
    uint16 remain_bullet
    uint16 enemy_sentry_hp
    uint16 our_sentry_hp
    uint16 our_base_hp
    uint8 first_blood
    float32 target_position_x
    float32 target_position_y
    int8 is_target_active
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new competition_info(null);
    if (msg.game_state !== undefined) {
      resolved.game_state = msg.game_state;
    }
    else {
      resolved.game_state = 0
    }

    if (msg.our_outpost_hp !== undefined) {
      resolved.our_outpost_hp = msg.our_outpost_hp;
    }
    else {
      resolved.our_outpost_hp = 0
    }

    if (msg.enemy_outpost_hp !== undefined) {
      resolved.enemy_outpost_hp = msg.enemy_outpost_hp;
    }
    else {
      resolved.enemy_outpost_hp = 0
    }

    if (msg.remain_bullet !== undefined) {
      resolved.remain_bullet = msg.remain_bullet;
    }
    else {
      resolved.remain_bullet = 0
    }

    if (msg.enemy_sentry_hp !== undefined) {
      resolved.enemy_sentry_hp = msg.enemy_sentry_hp;
    }
    else {
      resolved.enemy_sentry_hp = 0
    }

    if (msg.our_sentry_hp !== undefined) {
      resolved.our_sentry_hp = msg.our_sentry_hp;
    }
    else {
      resolved.our_sentry_hp = 0
    }

    if (msg.our_base_hp !== undefined) {
      resolved.our_base_hp = msg.our_base_hp;
    }
    else {
      resolved.our_base_hp = 0
    }

    if (msg.first_blood !== undefined) {
      resolved.first_blood = msg.first_blood;
    }
    else {
      resolved.first_blood = 0
    }

    if (msg.target_position_x !== undefined) {
      resolved.target_position_x = msg.target_position_x;
    }
    else {
      resolved.target_position_x = 0.0
    }

    if (msg.target_position_y !== undefined) {
      resolved.target_position_y = msg.target_position_y;
    }
    else {
      resolved.target_position_y = 0.0
    }

    if (msg.is_target_active !== undefined) {
      resolved.is_target_active = msg.is_target_active;
    }
    else {
      resolved.is_target_active = 0
    }

    return resolved;
    }
};

module.exports = competition_info;
