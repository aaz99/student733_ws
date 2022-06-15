; Auto-generated. Do not edit!


(cl:in-package test_service-srv)


;//! \htmlinclude test_service-request.msg.html

(cl:defclass <test_service-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:integer
    :initform 0)
   (b
    :reader b
    :initarg :b
    :type cl:integer
    :initform 0)
   (c
    :reader c
    :initarg :c
    :type cl:integer
    :initform 0))
)

(cl:defclass test_service-request (<test_service-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <test_service-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'test_service-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name test_service-srv:<test_service-request> is deprecated: use test_service-srv:test_service-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <test_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader test_service-srv:a-val is deprecated.  Use test_service-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <test_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader test_service-srv:b-val is deprecated.  Use test_service-srv:b instead.")
  (b m))

(cl:ensure-generic-function 'c-val :lambda-list '(m))
(cl:defmethod c-val ((m <test_service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader test_service-srv:c-val is deprecated.  Use test_service-srv:c instead.")
  (c m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <test_service-request>) ostream)
  "Serializes a message object of type '<test_service-request>"
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'b)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'c)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <test_service-request>) istream)
  "Deserializes a message object of type '<test_service-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'b) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'c) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<test_service-request>)))
  "Returns string type for a service object of type '<test_service-request>"
  "test_service/test_serviceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'test_service-request)))
  "Returns string type for a service object of type 'test_service-request"
  "test_service/test_serviceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<test_service-request>)))
  "Returns md5sum for a message object of type '<test_service-request>"
  "824d60fc841435159cb0442f45e7ceac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'test_service-request)))
  "Returns md5sum for a message object of type 'test_service-request"
  "824d60fc841435159cb0442f45e7ceac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<test_service-request>)))
  "Returns full string definition for message of type '<test_service-request>"
  (cl:format cl:nil "int64 a~%int64 b~%int64 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'test_service-request)))
  "Returns full string definition for message of type 'test_service-request"
  (cl:format cl:nil "int64 a~%int64 b~%int64 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <test_service-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <test_service-request>))
  "Converts a ROS message object to a list"
  (cl:list 'test_service-request
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
    (cl:cons ':c (c msg))
))
;//! \htmlinclude test_service-response.msg.html

(cl:defclass <test_service-response> (roslisp-msg-protocol:ros-message)
  ((square
    :reader square
    :initarg :square
    :type cl:float
    :initform 0.0))
)

(cl:defclass test_service-response (<test_service-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <test_service-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'test_service-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name test_service-srv:<test_service-response> is deprecated: use test_service-srv:test_service-response instead.")))

(cl:ensure-generic-function 'square-val :lambda-list '(m))
(cl:defmethod square-val ((m <test_service-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader test_service-srv:square-val is deprecated.  Use test_service-srv:square instead.")
  (square m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <test_service-response>) ostream)
  "Serializes a message object of type '<test_service-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'square))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <test_service-response>) istream)
  "Deserializes a message object of type '<test_service-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'square) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<test_service-response>)))
  "Returns string type for a service object of type '<test_service-response>"
  "test_service/test_serviceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'test_service-response)))
  "Returns string type for a service object of type 'test_service-response"
  "test_service/test_serviceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<test_service-response>)))
  "Returns md5sum for a message object of type '<test_service-response>"
  "824d60fc841435159cb0442f45e7ceac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'test_service-response)))
  "Returns md5sum for a message object of type 'test_service-response"
  "824d60fc841435159cb0442f45e7ceac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<test_service-response>)))
  "Returns full string definition for message of type '<test_service-response>"
  (cl:format cl:nil "float64 square~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'test_service-response)))
  "Returns full string definition for message of type 'test_service-response"
  (cl:format cl:nil "float64 square~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <test_service-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <test_service-response>))
  "Converts a ROS message object to a list"
  (cl:list 'test_service-response
    (cl:cons ':square (square msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'test_service)))
  'test_service-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'test_service)))
  'test_service-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'test_service)))
  "Returns string type for a service object of type '<test_service>"
  "test_service/test_service")