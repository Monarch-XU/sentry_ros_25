;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::ChassisMoveStatus)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'ChassisMoveStatus (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::CHASSISMOVESTATUS")
  (make-package "ROBOT_MSGS::CHASSISMOVESTATUS"))
(unless (find-package "ROBOT_MSGS::CHASSISMOVESTATUSREQUEST")
  (make-package "ROBOT_MSGS::CHASSISMOVESTATUSREQUEST"))
(unless (find-package "ROBOT_MSGS::CHASSISMOVESTATUSRESPONSE")
  (make-package "ROBOT_MSGS::CHASSISMOVESTATUSRESPONSE"))

(in-package "ROS")





(defclass robot_msgs::ChassisMoveStatusRequest
  :super ros::object
  :slots (_planner_state _speed_x_adjust _speed_y_adjust ))

(defmethod robot_msgs::ChassisMoveStatusRequest
  (:init
   (&key
    ((:planner_state __planner_state) 0)
    ((:speed_x_adjust __speed_x_adjust) 0.0)
    ((:speed_y_adjust __speed_y_adjust) 0.0)
    )
   (send-super :init)
   (setq _planner_state (round __planner_state))
   (setq _speed_x_adjust (float __speed_x_adjust))
   (setq _speed_y_adjust (float __speed_y_adjust))
   self)
  (:planner_state
   (&optional __planner_state)
   (if __planner_state (setq _planner_state __planner_state)) _planner_state)
  (:speed_x_adjust
   (&optional __speed_x_adjust)
   (if __speed_x_adjust (setq _speed_x_adjust __speed_x_adjust)) _speed_x_adjust)
  (:speed_y_adjust
   (&optional __speed_y_adjust)
   (if __speed_y_adjust (setq _speed_y_adjust __speed_y_adjust)) _speed_y_adjust)
  (:serialization-length
   ()
   (+
    ;; int32 _planner_state
    4
    ;; float32 _speed_x_adjust
    4
    ;; float32 _speed_y_adjust
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _planner_state
       (write-long _planner_state s)
     ;; float32 _speed_x_adjust
       (sys::poke _speed_x_adjust (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed_y_adjust
       (sys::poke _speed_y_adjust (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _planner_state
     (setq _planner_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _speed_x_adjust
     (setq _speed_x_adjust (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed_y_adjust
     (setq _speed_y_adjust (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass robot_msgs::ChassisMoveStatusResponse
  :super ros::object
  :slots (_result ))

(defmethod robot_msgs::ChassisMoveStatusResponse
  (:init
   (&key
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; int32 _result
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _result
       (write-long _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _result
     (setq _result (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass robot_msgs::ChassisMoveStatus
  :super ros::object
  :slots ())

(setf (get robot_msgs::ChassisMoveStatus :md5sum-) "0a8ab4662f6938ca464a048217cc826e")
(setf (get robot_msgs::ChassisMoveStatus :datatype-) "robot_msgs/ChassisMoveStatus")
(setf (get robot_msgs::ChassisMoveStatus :request) robot_msgs::ChassisMoveStatusRequest)
(setf (get robot_msgs::ChassisMoveStatus :response) robot_msgs::ChassisMoveStatusResponse)

(defmethod robot_msgs::ChassisMoveStatusRequest
  (:response () (instance robot_msgs::ChassisMoveStatusResponse :init)))

(setf (get robot_msgs::ChassisMoveStatusRequest :md5sum-) "0a8ab4662f6938ca464a048217cc826e")
(setf (get robot_msgs::ChassisMoveStatusRequest :datatype-) "robot_msgs/ChassisMoveStatusRequest")
(setf (get robot_msgs::ChassisMoveStatusRequest :definition-)
      "# 客户端请求时设置状态
int32 planner_state
float32 speed_x_adjust
float32 speed_y_adjust
---
int32 result

")

(setf (get robot_msgs::ChassisMoveStatusResponse :md5sum-) "0a8ab4662f6938ca464a048217cc826e")
(setf (get robot_msgs::ChassisMoveStatusResponse :datatype-) "robot_msgs/ChassisMoveStatusResponse")
(setf (get robot_msgs::ChassisMoveStatusResponse :definition-)
      "# 客户端请求时设置状态
int32 planner_state
float32 speed_x_adjust
float32 speed_y_adjust
---
int32 result

")



(provide :robot_msgs/ChassisMoveStatus "0a8ab4662f6938ca464a048217cc826e")


