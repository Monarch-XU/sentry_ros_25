
(cl:in-package :asdf)

(defsystem "robot_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ChassisMoveStatus" :depends-on ("_package_ChassisMoveStatus"))
    (:file "_package_ChassisMoveStatus" :depends-on ("_package"))
  ))