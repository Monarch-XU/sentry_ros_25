; Auto-generated. Do not edit!


(cl:in-package robot_msgs-srv)


;//! \htmlinclude ChassisMoveStatus-request.msg.html

(cl:defclass <ChassisMoveStatus-request> (roslisp-msg-protocol:ros-message)
  ((planner_state
    :reader planner_state
    :initarg :planner_state
    :type cl:integer
    :initform 0)
   (speed_x_adjust
    :reader speed_x_adjust
    :initarg :speed_x_adjust
    :type cl:float
    :initform 0.0)
   (speed_y_adjust
    :reader speed_y_adjust
    :initarg :speed_y_adjust
    :type cl:float
    :initform 0.0))
)

(cl:defclass ChassisMoveStatus-request (<ChassisMoveStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChassisMoveStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChassisMoveStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msgs-srv:<ChassisMoveStatus-request> is deprecated: use robot_msgs-srv:ChassisMoveStatus-request instead.")))

(cl:ensure-generic-function 'planner_state-val :lambda-list '(m))
(cl:defmethod planner_state-val ((m <ChassisMoveStatus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-srv:planner_state-val is deprecated.  Use robot_msgs-srv:planner_state instead.")
  (planner_state m))

(cl:ensure-generic-function 'speed_x_adjust-val :lambda-list '(m))
(cl:defmethod speed_x_adjust-val ((m <ChassisMoveStatus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-srv:speed_x_adjust-val is deprecated.  Use robot_msgs-srv:speed_x_adjust instead.")
  (speed_x_adjust m))

(cl:ensure-generic-function 'speed_y_adjust-val :lambda-list '(m))
(cl:defmethod speed_y_adjust-val ((m <ChassisMoveStatus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-srv:speed_y_adjust-val is deprecated.  Use robot_msgs-srv:speed_y_adjust instead.")
  (speed_y_adjust m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChassisMoveStatus-request>) ostream)
  "Serializes a message object of type '<ChassisMoveStatus-request>"
  (cl:let* ((signed (cl:slot-value msg 'planner_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed_x_adjust))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed_y_adjust))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChassisMoveStatus-request>) istream)
  "Deserializes a message object of type '<ChassisMoveStatus-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'planner_state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_x_adjust) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_y_adjust) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChassisMoveStatus-request>)))
  "Returns string type for a service object of type '<ChassisMoveStatus-request>"
  "robot_msgs/ChassisMoveStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChassisMoveStatus-request)))
  "Returns string type for a service object of type 'ChassisMoveStatus-request"
  "robot_msgs/ChassisMoveStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChassisMoveStatus-request>)))
  "Returns md5sum for a message object of type '<ChassisMoveStatus-request>"
  "0a8ab4662f6938ca464a048217cc826e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChassisMoveStatus-request)))
  "Returns md5sum for a message object of type 'ChassisMoveStatus-request"
  "0a8ab4662f6938ca464a048217cc826e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChassisMoveStatus-request>)))
  "Returns full string definition for message of type '<ChassisMoveStatus-request>"
  (cl:format cl:nil "# 客户端请求时设置状态~%int32 planner_state~%float32 speed_x_adjust~%float32 speed_y_adjust~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChassisMoveStatus-request)))
  "Returns full string definition for message of type 'ChassisMoveStatus-request"
  (cl:format cl:nil "# 客户端请求时设置状态~%int32 planner_state~%float32 speed_x_adjust~%float32 speed_y_adjust~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChassisMoveStatus-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChassisMoveStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ChassisMoveStatus-request
    (cl:cons ':planner_state (planner_state msg))
    (cl:cons ':speed_x_adjust (speed_x_adjust msg))
    (cl:cons ':speed_y_adjust (speed_y_adjust msg))
))
;//! \htmlinclude ChassisMoveStatus-response.msg.html

(cl:defclass <ChassisMoveStatus-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass ChassisMoveStatus-response (<ChassisMoveStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChassisMoveStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChassisMoveStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msgs-srv:<ChassisMoveStatus-response> is deprecated: use robot_msgs-srv:ChassisMoveStatus-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <ChassisMoveStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-srv:result-val is deprecated.  Use robot_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChassisMoveStatus-response>) ostream)
  "Serializes a message object of type '<ChassisMoveStatus-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChassisMoveStatus-response>) istream)
  "Deserializes a message object of type '<ChassisMoveStatus-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChassisMoveStatus-response>)))
  "Returns string type for a service object of type '<ChassisMoveStatus-response>"
  "robot_msgs/ChassisMoveStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChassisMoveStatus-response)))
  "Returns string type for a service object of type 'ChassisMoveStatus-response"
  "robot_msgs/ChassisMoveStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChassisMoveStatus-response>)))
  "Returns md5sum for a message object of type '<ChassisMoveStatus-response>"
  "0a8ab4662f6938ca464a048217cc826e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChassisMoveStatus-response)))
  "Returns md5sum for a message object of type 'ChassisMoveStatus-response"
  "0a8ab4662f6938ca464a048217cc826e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChassisMoveStatus-response>)))
  "Returns full string definition for message of type '<ChassisMoveStatus-response>"
  (cl:format cl:nil "int32 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChassisMoveStatus-response)))
  "Returns full string definition for message of type 'ChassisMoveStatus-response"
  (cl:format cl:nil "int32 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChassisMoveStatus-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChassisMoveStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ChassisMoveStatus-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ChassisMoveStatus)))
  'ChassisMoveStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ChassisMoveStatus)))
  'ChassisMoveStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChassisMoveStatus)))
  "Returns string type for a service object of type '<ChassisMoveStatus>"
  "robot_msgs/ChassisMoveStatus")