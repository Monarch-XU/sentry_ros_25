; Auto-generated. Do not edit!


(cl:in-package robot_msgs-msg)


;//! \htmlinclude attack_base.msg.html

(cl:defclass <attack_base> (roslisp-msg-protocol:ros-message)
  ((yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (flag
    :reader flag
    :initarg :flag
    :type cl:fixnum
    :initform 0))
)

(cl:defclass attack_base (<attack_base>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <attack_base>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'attack_base)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msgs-msg:<attack_base> is deprecated: use robot_msgs-msg:attack_base instead.")))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <attack_base>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:yaw-val is deprecated.  Use robot_msgs-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <attack_base>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:flag-val is deprecated.  Use robot_msgs-msg:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <attack_base>) ostream)
  "Serializes a message object of type '<attack_base>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'flag)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <attack_base>) istream)
  "Deserializes a message object of type '<attack_base>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'flag) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<attack_base>)))
  "Returns string type for a message object of type '<attack_base>"
  "robot_msgs/attack_base")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'attack_base)))
  "Returns string type for a message object of type 'attack_base"
  "robot_msgs/attack_base")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<attack_base>)))
  "Returns md5sum for a message object of type '<attack_base>"
  "2a64b3102681d4388f983b59c1ae894d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'attack_base)))
  "Returns md5sum for a message object of type 'attack_base"
  "2a64b3102681d4388f983b59c1ae894d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<attack_base>)))
  "Returns full string definition for message of type '<attack_base>"
  (cl:format cl:nil "float32 yaw~%int8 flag~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'attack_base)))
  "Returns full string definition for message of type 'attack_base"
  (cl:format cl:nil "float32 yaw~%int8 flag~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <attack_base>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <attack_base>))
  "Converts a ROS message object to a list"
  (cl:list 'attack_base
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':flag (flag msg))
))
