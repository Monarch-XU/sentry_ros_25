;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::barrel)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'barrel (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::BARREL")
  (make-package "ROBOT_MSGS::BARREL"))

(in-package "ROS")
;;//! \htmlinclude barrel.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robot_msgs::barrel
  :super ros::object
  :slots (_header _num _id _grade _yaw _x _y _z ))

(defmethod robot_msgs::barrel
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:num __num) 0)
    ((:id __id) 0)
    ((:grade __grade) 0)
    ((:yaw __yaw) 0.0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _num (round __num))
   (setq _id (round __id))
   (setq _grade (round __grade))
   (setq _yaw (float __yaw))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:grade
   (&optional __grade)
   (if __grade (setq _grade __grade)) _grade)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int8 _num
    1
    ;; int8 _id
    1
    ;; int32 _grade
    4
    ;; float32 _yaw
    4
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int8 _num
       (write-byte _num s)
     ;; int8 _id
       (write-byte _id s)
     ;; int32 _grade
       (write-long _grade s)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int8 _num
     (setq _num (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _num 127) (setq _num (- _num 256)))
   ;; int8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _id 127) (setq _id (- _id 256)))
   ;; int32 _grade
     (setq _grade (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robot_msgs::barrel :md5sum-) "63bd136c4dab05090a6031f8df03e306")
(setf (get robot_msgs::barrel :datatype-) "robot_msgs/barrel")
(setf (get robot_msgs::barrel :definition-)
      "Header header
int8 num
int8 id
int32 grade
float32 yaw
float32 x
float32 y
float32 z
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

")



(provide :robot_msgs/barrel "63bd136c4dab05090a6031f8df03e306")


