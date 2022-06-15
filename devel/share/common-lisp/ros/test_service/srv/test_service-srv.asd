
(cl:in-package :asdf)

(defsystem "test_service-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "test_service" :depends-on ("_package_test_service"))
    (:file "_package_test_service" :depends-on ("_package"))
  ))