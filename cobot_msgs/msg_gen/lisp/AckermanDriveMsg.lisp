; Auto-generated. Do not edit!


(cl:in-package cobot_msgs-msg)


;//! \htmlinclude AckermanDriveMsg.msg.html

(cl:defclass <AckermanDriveMsg> (roslisp-msg-protocol:ros-message)
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
   (R
    :reader R
    :initarg :R
    :type cl:float
    :initform 0.0))
)

(cl:defclass AckermanDriveMsg (<AckermanDriveMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AckermanDriveMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AckermanDriveMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cobot_msgs-msg:<AckermanDriveMsg> is deprecated: use cobot_msgs-msg:AckermanDriveMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AckermanDriveMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cobot_msgs-msg:header-val is deprecated.  Use cobot_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <AckermanDriveMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cobot_msgs-msg:v-val is deprecated.  Use cobot_msgs-msg:v instead.")
  (v m))

(cl:ensure-generic-function 'R-val :lambda-list '(m))
(cl:defmethod R-val ((m <AckermanDriveMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cobot_msgs-msg:R-val is deprecated.  Use cobot_msgs-msg:R instead.")
  (R m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AckermanDriveMsg>) ostream)
  "Serializes a message object of type '<AckermanDriveMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'R))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AckermanDriveMsg>) istream)
  "Deserializes a message object of type '<AckermanDriveMsg>"
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
    (cl:setf (cl:slot-value msg 'R) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AckermanDriveMsg>)))
  "Returns string type for a message object of type '<AckermanDriveMsg>"
  "cobot_msgs/AckermanDriveMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AckermanDriveMsg)))
  "Returns string type for a message object of type 'AckermanDriveMsg"
  "cobot_msgs/AckermanDriveMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AckermanDriveMsg>)))
  "Returns md5sum for a message object of type '<AckermanDriveMsg>"
  "b46e25bb9c7b4a4f6d687cffd13da65c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AckermanDriveMsg)))
  "Returns md5sum for a message object of type 'AckermanDriveMsg"
  "b46e25bb9c7b4a4f6d687cffd13da65c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AckermanDriveMsg>)))
  "Returns full string definition for message of type '<AckermanDriveMsg>"
  (cl:format cl:nil "Header header~%~%# linear velocity command in the forward direction [m/s]~%float32 v~%~%# Inverse Turning Radius [m]~%float32 R~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AckermanDriveMsg)))
  "Returns full string definition for message of type 'AckermanDriveMsg"
  (cl:format cl:nil "Header header~%~%# linear velocity command in the forward direction [m/s]~%float32 v~%~%# Inverse Turning Radius [m]~%float32 R~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AckermanDriveMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AckermanDriveMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'AckermanDriveMsg
    (cl:cons ':header (header msg))
    (cl:cons ':v (v msg))
    (cl:cons ':R (R msg))
))
