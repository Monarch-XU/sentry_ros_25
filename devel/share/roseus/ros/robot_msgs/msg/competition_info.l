;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::competition_info)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'competition_info (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::COMPETITION_INFO")
  (make-package "ROBOT_MSGS::COMPETITION_INFO"))

(in-package "ROS")
;;//! \htmlinclude competition_info.msg.html


(defclass robot_msgs::competition_info
  :super ros::object
  :slots (_game_state _our_outpost_hp _enemy_outpost_hp _remain_bullet _enemy_sentry_hp _our_sentry_hp _our_base_hp _first_blood _target_position_x _target_position_y _is_target_active ))

(defmethod robot_msgs::competition_info
  (:init
   (&key
    ((:game_state __game_state) 0)
    ((:our_outpost_hp __our_outpost_hp) 0)
    ((:enemy_outpost_hp __enemy_outpost_hp) 0)
    ((:remain_bullet __remain_bullet) 0)
    ((:enemy_sentry_hp __enemy_sentry_hp) 0)
    ((:our_sentry_hp __our_sentry_hp) 0)
    ((:our_base_hp __our_base_hp) 0)
    ((:first_blood __first_blood) 0)
    ((:target_position_x __target_position_x) 0.0)
    ((:target_position_y __target_position_y) 0.0)
    ((:is_target_active __is_target_active) 0)
    )
   (send-super :init)
   (setq _game_state (round __game_state))
   (setq _our_outpost_hp (round __our_outpost_hp))
   (setq _enemy_outpost_hp (round __enemy_outpost_hp))
   (setq _remain_bullet (round __remain_bullet))
   (setq _enemy_sentry_hp (round __enemy_sentry_hp))
   (setq _our_sentry_hp (round __our_sentry_hp))
   (setq _our_base_hp (round __our_base_hp))
   (setq _first_blood (round __first_blood))
   (setq _target_position_x (float __target_position_x))
   (setq _target_position_y (float __target_position_y))
   (setq _is_target_active (round __is_target_active))
   self)
  (:game_state
   (&optional __game_state)
   (if __game_state (setq _game_state __game_state)) _game_state)
  (:our_outpost_hp
   (&optional __our_outpost_hp)
   (if __our_outpost_hp (setq _our_outpost_hp __our_outpost_hp)) _our_outpost_hp)
  (:enemy_outpost_hp
   (&optional __enemy_outpost_hp)
   (if __enemy_outpost_hp (setq _enemy_outpost_hp __enemy_outpost_hp)) _enemy_outpost_hp)
  (:remain_bullet
   (&optional __remain_bullet)
   (if __remain_bullet (setq _remain_bullet __remain_bullet)) _remain_bullet)
  (:enemy_sentry_hp
   (&optional __enemy_sentry_hp)
   (if __enemy_sentry_hp (setq _enemy_sentry_hp __enemy_sentry_hp)) _enemy_sentry_hp)
  (:our_sentry_hp
   (&optional __our_sentry_hp)
   (if __our_sentry_hp (setq _our_sentry_hp __our_sentry_hp)) _our_sentry_hp)
  (:our_base_hp
   (&optional __our_base_hp)
   (if __our_base_hp (setq _our_base_hp __our_base_hp)) _our_base_hp)
  (:first_blood
   (&optional __first_blood)
   (if __first_blood (setq _first_blood __first_blood)) _first_blood)
  (:target_position_x
   (&optional __target_position_x)
   (if __target_position_x (setq _target_position_x __target_position_x)) _target_position_x)
  (:target_position_y
   (&optional __target_position_y)
   (if __target_position_y (setq _target_position_y __target_position_y)) _target_position_y)
  (:is_target_active
   (&optional __is_target_active)
   (if __is_target_active (setq _is_target_active __is_target_active)) _is_target_active)
  (:serialization-length
   ()
   (+
    ;; uint8 _game_state
    1
    ;; uint16 _our_outpost_hp
    2
    ;; uint16 _enemy_outpost_hp
    2
    ;; uint16 _remain_bullet
    2
    ;; uint16 _enemy_sentry_hp
    2
    ;; uint16 _our_sentry_hp
    2
    ;; uint16 _our_base_hp
    2
    ;; uint8 _first_blood
    1
    ;; float32 _target_position_x
    4
    ;; float32 _target_position_y
    4
    ;; int8 _is_target_active
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _game_state
       (write-byte _game_state s)
     ;; uint16 _our_outpost_hp
       (write-word _our_outpost_hp s)
     ;; uint16 _enemy_outpost_hp
       (write-word _enemy_outpost_hp s)
     ;; uint16 _remain_bullet
       (write-word _remain_bullet s)
     ;; uint16 _enemy_sentry_hp
       (write-word _enemy_sentry_hp s)
     ;; uint16 _our_sentry_hp
       (write-word _our_sentry_hp s)
     ;; uint16 _our_base_hp
       (write-word _our_base_hp s)
     ;; uint8 _first_blood
       (write-byte _first_blood s)
     ;; float32 _target_position_x
       (sys::poke _target_position_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _target_position_y
       (sys::poke _target_position_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _is_target_active
       (write-byte _is_target_active s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _game_state
     (setq _game_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _our_outpost_hp
     (setq _our_outpost_hp (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _enemy_outpost_hp
     (setq _enemy_outpost_hp (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _remain_bullet
     (setq _remain_bullet (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _enemy_sentry_hp
     (setq _enemy_sentry_hp (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _our_sentry_hp
     (setq _our_sentry_hp (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _our_base_hp
     (setq _our_base_hp (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _first_blood
     (setq _first_blood (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _target_position_x
     (setq _target_position_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _target_position_y
     (setq _target_position_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _is_target_active
     (setq _is_target_active (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _is_target_active 127) (setq _is_target_active (- _is_target_active 256)))
   ;;
   self)
  )

(setf (get robot_msgs::competition_info :md5sum-) "0126e094b04a5313899b587ef4b04645")
(setf (get robot_msgs::competition_info :datatype-) "robot_msgs/competition_info")
(setf (get robot_msgs::competition_info :definition-)
      "uint8 game_state
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
")



(provide :robot_msgs/competition_info "0126e094b04a5313899b587ef4b04645")


