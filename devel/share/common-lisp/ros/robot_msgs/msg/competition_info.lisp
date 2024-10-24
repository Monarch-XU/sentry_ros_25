; Auto-generated. Do not edit!


(cl:in-package robot_msgs-msg)


;//! \htmlinclude competition_info.msg.html

(cl:defclass <competition_info> (roslisp-msg-protocol:ros-message)
  ((game_state
    :reader game_state
    :initarg :game_state
    :type cl:fixnum
    :initform 0)
   (our_outpost_hp
    :reader our_outpost_hp
    :initarg :our_outpost_hp
    :type cl:fixnum
    :initform 0)
   (enemy_outpost_hp
    :reader enemy_outpost_hp
    :initarg :enemy_outpost_hp
    :type cl:fixnum
    :initform 0)
   (remain_bullet
    :reader remain_bullet
    :initarg :remain_bullet
    :type cl:fixnum
    :initform 0)
   (enemy_sentry_hp
    :reader enemy_sentry_hp
    :initarg :enemy_sentry_hp
    :type cl:fixnum
    :initform 0)
   (our_sentry_hp
    :reader our_sentry_hp
    :initarg :our_sentry_hp
    :type cl:fixnum
    :initform 0)
   (our_base_hp
    :reader our_base_hp
    :initarg :our_base_hp
    :type cl:fixnum
    :initform 0)
   (first_blood
    :reader first_blood
    :initarg :first_blood
    :type cl:fixnum
    :initform 0)
   (target_position_x
    :reader target_position_x
    :initarg :target_position_x
    :type cl:float
    :initform 0.0)
   (target_position_y
    :reader target_position_y
    :initarg :target_position_y
    :type cl:float
    :initform 0.0)
   (is_target_active
    :reader is_target_active
    :initarg :is_target_active
    :type cl:fixnum
    :initform 0))
)

(cl:defclass competition_info (<competition_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <competition_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'competition_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msgs-msg:<competition_info> is deprecated: use robot_msgs-msg:competition_info instead.")))

(cl:ensure-generic-function 'game_state-val :lambda-list '(m))
(cl:defmethod game_state-val ((m <competition_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:game_state-val is deprecated.  Use robot_msgs-msg:game_state instead.")
  (game_state m))

(cl:ensure-generic-function 'our_outpost_hp-val :lambda-list '(m))
(cl:defmethod our_outpost_hp-val ((m <competition_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:our_outpost_hp-val is deprecated.  Use robot_msgs-msg:our_outpost_hp instead.")
  (our_outpost_hp m))

(cl:ensure-generic-function 'enemy_outpost_hp-val :lambda-list '(m))
(cl:defmethod enemy_outpost_hp-val ((m <competition_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:enemy_outpost_hp-val is deprecated.  Use robot_msgs-msg:enemy_outpost_hp instead.")
  (enemy_outpost_hp m))

(cl:ensure-generic-function 'remain_bullet-val :lambda-list '(m))
(cl:defmethod remain_bullet-val ((m <competition_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:remain_bullet-val is deprecated.  Use robot_msgs-msg:remain_bullet instead.")
  (remain_bullet m))

(cl:ensure-generic-function 'enemy_sentry_hp-val :lambda-list '(m))
(cl:defmethod enemy_sentry_hp-val ((m <competition_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:enemy_sentry_hp-val is deprecated.  Use robot_msgs-msg:enemy_sentry_hp instead.")
  (enemy_sentry_hp m))

(cl:ensure-generic-function 'our_sentry_hp-val :lambda-list '(m))
(cl:defmethod our_sentry_hp-val ((m <competition_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:our_sentry_hp-val is deprecated.  Use robot_msgs-msg:our_sentry_hp instead.")
  (our_sentry_hp m))

(cl:ensure-generic-function 'our_base_hp-val :lambda-list '(m))
(cl:defmethod our_base_hp-val ((m <competition_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:our_base_hp-val is deprecated.  Use robot_msgs-msg:our_base_hp instead.")
  (our_base_hp m))

(cl:ensure-generic-function 'first_blood-val :lambda-list '(m))
(cl:defmethod first_blood-val ((m <competition_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:first_blood-val is deprecated.  Use robot_msgs-msg:first_blood instead.")
  (first_blood m))

(cl:ensure-generic-function 'target_position_x-val :lambda-list '(m))
(cl:defmethod target_position_x-val ((m <competition_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:target_position_x-val is deprecated.  Use robot_msgs-msg:target_position_x instead.")
  (target_position_x m))

(cl:ensure-generic-function 'target_position_y-val :lambda-list '(m))
(cl:defmethod target_position_y-val ((m <competition_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:target_position_y-val is deprecated.  Use robot_msgs-msg:target_position_y instead.")
  (target_position_y m))

(cl:ensure-generic-function 'is_target_active-val :lambda-list '(m))
(cl:defmethod is_target_active-val ((m <competition_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:is_target_active-val is deprecated.  Use robot_msgs-msg:is_target_active instead.")
  (is_target_active m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <competition_info>) ostream)
  "Serializes a message object of type '<competition_info>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'game_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'our_outpost_hp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'our_outpost_hp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'enemy_outpost_hp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'enemy_outpost_hp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'remain_bullet)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'remain_bullet)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'enemy_sentry_hp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'enemy_sentry_hp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'our_sentry_hp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'our_sentry_hp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'our_base_hp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'our_base_hp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'first_blood)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_position_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_position_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'is_target_active)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <competition_info>) istream)
  "Deserializes a message object of type '<competition_info>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'game_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'our_outpost_hp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'our_outpost_hp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'enemy_outpost_hp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'enemy_outpost_hp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'remain_bullet)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'remain_bullet)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'enemy_sentry_hp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'enemy_sentry_hp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'our_sentry_hp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'our_sentry_hp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'our_base_hp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'our_base_hp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'first_blood)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_position_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_position_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'is_target_active) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<competition_info>)))
  "Returns string type for a message object of type '<competition_info>"
  "robot_msgs/competition_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'competition_info)))
  "Returns string type for a message object of type 'competition_info"
  "robot_msgs/competition_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<competition_info>)))
  "Returns md5sum for a message object of type '<competition_info>"
  "0126e094b04a5313899b587ef4b04645")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'competition_info)))
  "Returns md5sum for a message object of type 'competition_info"
  "0126e094b04a5313899b587ef4b04645")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<competition_info>)))
  "Returns full string definition for message of type '<competition_info>"
  (cl:format cl:nil "uint8 game_state~%uint16 our_outpost_hp~%uint16 enemy_outpost_hp~%uint16 remain_bullet~%uint16 enemy_sentry_hp~%uint16 our_sentry_hp~%uint16 our_base_hp~%uint8 first_blood~%float32 target_position_x~%float32 target_position_y~%int8 is_target_active~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'competition_info)))
  "Returns full string definition for message of type 'competition_info"
  (cl:format cl:nil "uint8 game_state~%uint16 our_outpost_hp~%uint16 enemy_outpost_hp~%uint16 remain_bullet~%uint16 enemy_sentry_hp~%uint16 our_sentry_hp~%uint16 our_base_hp~%uint8 first_blood~%float32 target_position_x~%float32 target_position_y~%int8 is_target_active~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <competition_info>))
  (cl:+ 0
     1
     2
     2
     2
     2
     2
     2
     1
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <competition_info>))
  "Converts a ROS message object to a list"
  (cl:list 'competition_info
    (cl:cons ':game_state (game_state msg))
    (cl:cons ':our_outpost_hp (our_outpost_hp msg))
    (cl:cons ':enemy_outpost_hp (enemy_outpost_hp msg))
    (cl:cons ':remain_bullet (remain_bullet msg))
    (cl:cons ':enemy_sentry_hp (enemy_sentry_hp msg))
    (cl:cons ':our_sentry_hp (our_sentry_hp msg))
    (cl:cons ':our_base_hp (our_base_hp msg))
    (cl:cons ':first_blood (first_blood msg))
    (cl:cons ':target_position_x (target_position_x msg))
    (cl:cons ':target_position_y (target_position_y msg))
    (cl:cons ':is_target_active (is_target_active msg))
))
