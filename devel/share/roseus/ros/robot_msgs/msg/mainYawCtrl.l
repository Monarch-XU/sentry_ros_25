;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::mainYawCtrl)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'mainYawCtrl (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::MAINYAWCTRL")
  (make-package "ROBOT_MSGS::MAINYAWCTRL"))

(in-package "ROS")
;;//! \htmlinclude mainYawCtrl.msg.html


(defclass robot_msgs::mainYawCtrl
  :super ros::object
  :slots (_yaw _id ))

(defmethod robot_msgs::mainYawCtrl
  (:init
   (&key
    ((:yaw __yaw) 0.0)
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _yaw (float __yaw))
   (setq _id (round __id))
   self)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; float32 _yaw
    4
    ;; int8 _id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _id
       (write-byte _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _id 127) (setq _id (- _id 256)))
   ;;
   self)
  )

(setf (get robot_msgs::mainYawCtrl :md5sum-) "c12ee215529067b9c49a63eae551cbc9")
(setf (get robot_msgs::mainYawCtrl :datatype-) "robot_msgs/mainYawCtrl")
(setf (get robot_msgs::mainYawCtrl :definition-)
      "float32 yaw
int8 id
")



(provide :robot_msgs/mainYawCtrl "c12ee215529067b9c49a63eae551cbc9")


