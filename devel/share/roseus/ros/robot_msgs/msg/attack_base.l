;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::attack_base)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'attack_base (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::ATTACK_BASE")
  (make-package "ROBOT_MSGS::ATTACK_BASE"))

(in-package "ROS")
;;//! \htmlinclude attack_base.msg.html


(defclass robot_msgs::attack_base
  :super ros::object
  :slots (_yaw _flag ))

(defmethod robot_msgs::attack_base
  (:init
   (&key
    ((:yaw __yaw) 0.0)
    ((:flag __flag) 0)
    )
   (send-super :init)
   (setq _yaw (float __yaw))
   (setq _flag (round __flag))
   self)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:serialization-length
   ()
   (+
    ;; float32 _yaw
    4
    ;; int8 _flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _flag
       (write-byte _flag s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _flag
     (setq _flag (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _flag 127) (setq _flag (- _flag 256)))
   ;;
   self)
  )

(setf (get robot_msgs::attack_base :md5sum-) "2a64b3102681d4388f983b59c1ae894d")
(setf (get robot_msgs::attack_base :datatype-) "robot_msgs/attack_base")
(setf (get robot_msgs::attack_base :definition-)
      "float32 yaw
int8 flag
")



(provide :robot_msgs/attack_base "2a64b3102681d4388f983b59c1ae894d")


