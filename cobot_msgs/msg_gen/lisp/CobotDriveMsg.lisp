; Auto-generated. Do not edit!


(cl:in-package cobot_msgs-msg)


;//! \htmlinclude CobotDriveMsg.msg.html

(cl:defclass <CobotDriveMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (v
    :reader v
    :initarg :v
    :type cl:float
    :initform 0.0)
   (w
    :reader w
    :initarg :w
    :type cl:float
    :initform 0.0))
)

(cl:defclass CobotDriveMsg (<CobotDriveMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CobotDriveMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CobotDriveMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cobot_msgs-msg:<CobotDriveMsg> is deprecated: use cobot_msgs-msg:CobotDriveMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CobotDriveMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cobot_msgs-msg:header-val is deprecated.  Use cobot_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <CobotDriveMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cobot_msgs-msg:v-val is deprecated.  Use cobot_msgs-msg:v instead.")
  (v m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <CobotDriveMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cobot_msgs-msg:w-val is deprecated.  Use cobot_msgs-msg:w instead.")
  (w m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CobotDriveMsg>) ostream)
  "Serializes a message object of type '<CobotDriveMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CobotDriveMsg>) istream)
  "Deserializes a message object of type '<CobotDriveMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'w) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CobotDriveMsg>)))
  "Returns string type for a message object of type '<CobotDriveMsg>"
  "cobot_msgs/CobotDriveMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CobotDriveMsg)))
  "Returns string type for a message object of type 'CobotDriveMsg"
  "cobot_msgs/CobotDriveMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CobotDriveMsg>)))
  "Returns md5sum for a message object of type '<CobotDriveMsg>"
  "c9739f01512ce85d8ac1ccdd6bde650b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CobotDriveMsg)))
  "Returns md5sum for a message object of type 'CobotDriveMsg"
  "c9739f01512ce85d8ac1ccdd6bde650b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CobotDriveMsg>)))
  "Returns full string definition for message of type '<CobotDriveMsg>"
  (cl:format cl:nil "Header header~%~%# linear velocity command in the forward direction [m/s]~%float32 v~%~%# angular velocity command [rad/s]~%float32 w~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CobotDriveMsg)))
  "Returns full string definition for message of type 'CobotDriveMsg"
  (cl:format cl:nil "Header header~%~%# linear velocity command in the forward direction [m/s]~%float32 v~%~%# angular velocity command [rad/s]~%float32 w~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CobotDriveMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CobotDriveMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'CobotDriveMsg
    (cl:cons ':header (header msg))
    (cl:cons ':v (v msg))
    (cl:cons ':w (w msg))
))
