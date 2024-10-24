; Auto-generated. Do not edit!


(cl:in-package robot_msgs-msg)


;//! \htmlinclude mainYawCtrl.msg.html

(cl:defclass <mainYawCtrl> (roslisp-msg-protocol:ros-message)
  ((yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass mainYawCtrl (<mainYawCtrl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mainYawCtrl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mainYawCtrl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msgs-msg:<mainYawCtrl> is deprecated: use robot_msgs-msg:mainYawCtrl instead.")))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <mainYawCtrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:yaw-val is deprecated.  Use robot_msgs-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <mainYawCtrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:id-val is deprecated.  Use robot_msgs-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mainYawCtrl>) ostream)
  "Serializes a message object of type '<mainYawCtrl>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mainYawCtrl>) istream)
  "Deserializes a message object of type '<mainYawCtrl>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mainYawCtrl>)))
  "Returns string type for a message object of type '<mainYawCtrl>"
  "robot_msgs/mainYawCtrl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mainYawCtrl)))
  "Returns string type for a message object of type 'mainYawCtrl"
  "robot_msgs/mainYawCtrl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mainYawCtrl>)))
  "Returns md5sum for a message object of type '<mainYawCtrl>"
  "c12ee215529067b9c49a63eae551cbc9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mainYawCtrl)))
  "Returns md5sum for a message object of type 'mainYawCtrl"
  "c12ee215529067b9c49a63eae551cbc9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mainYawCtrl>)))
  "Returns full string definition for message of type '<mainYawCtrl>"
  (cl:format cl:nil "float32 yaw~%int8 id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mainYawCtrl)))
  "Returns full string definition for message of type 'mainYawCtrl"
  (cl:format cl:nil "float32 yaw~%int8 id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mainYawCtrl>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mainYawCtrl>))
  "Converts a ROS message object to a list"
  (cl:list 'mainYawCtrl
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':id (id msg))
))
