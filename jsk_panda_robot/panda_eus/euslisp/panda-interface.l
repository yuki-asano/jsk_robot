(require :robot-interface "package://pr2eus/robot-interface.l")
(require :franka "package://panda_eus/models/panda.l")

(ros::roseus-add-msgs "franka_msgs")
(ros::roseus-add-msgs "franka_gripper")

(defclass panda-robot-interface
  :super robot-interface
  :slots (gripper-action gazebop
          error-recovery-act error
          gripper-grasp-action gripper-move-action gripper-homing-action gripper-stop-action
          )
  )
(defmethod panda-robot-interface
  (:init
   (&rest args &key ((:controller-timeout ct) nil))
   (prog1
       (send-super* :init :robot panda-robot
                    :joint-states-topic "panda/joint_states"
                    :controller-timeout ct args)
     ;; check whather the environement is Gazebo
     (setq gazebop (numberp (ros::get-param "/gazebo/time_step")))
     (if gazebop (ros::ros-warn "Using Gazebo environment"))

     ;; Controller setting for Gazebo environment
     (when gazebop
       (ros::subscribe "/lhand_controller/state" control_msgs::JointTrajectoryControllerState
                       #'send self :gripper-state-callback :larm :groupname groupname)
       (setq gripper-action (instance ros::simple-action-client :init
                                        "/lhand_controller/follow_joint_trajectory"
                                        control_msgs::FollowJointTrajectoryAction))
       (dolist (action (list gripper-action))
         (unless (and joint-action-enable (send action :wait-for-server 3))
           (setq joint-action-enable nil)
           (ros::ros-warn "~A is not respond" action)
           (return)))
       ) ;; when gazebo
     (unless gazebop
       (ros::ros-warn "real franka environemt interface is not impelemented")) ;; TODO

     ;; for error recovery
     (ros::create-nodehandle "error_group")
     (ros::subscribe "/panda/larm/has_error" std_msgs::Bool
                     #'send self :callback-larm-error 1 :groupname "error_group")
     (setq error-recovery-act (instance ros::simple-action-client :init
                                        "/panda/error_recovery"
                                        franka_msgs::ErrorRecoveryAction
                                        :groupname "error_group"
                                        ))
     ;; actions for gripper
     (setq gripper-grasp-action
       (instance ros::simple-action-client :init
             "/franka_gripper/grasp"
             franka_gripper::GraspAction))
     (setq gripper-homing-action
       (instance ros::simple-action-client :init
             "/franka_gripper/homing"
             franka_gripper::HomingAction))
     (setq gripper-move-action
       (instance ros::simple-action-client :init
             "/franka_gripper/move"
             franka_gripper::MoveAction))
     (setq gripper-stop-action
       (instance ros::simple-action-client :init
             "/franka_gripper/stop"
             franka_gripper::StopAction))
     ))
  (:default-controller
   ()
   (list
    (list
     (cons :controller-action "/panda/panda_effort_joint_trajectory_controller/follow_joint_trajectory")
     (cons :controller-state  "/panda/panda_effort_joint_trajectory_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (send-all (send robot :joint-list) :name))
     )))
  #|
  (:default-controller ;; for gazebo
   ()
   (list
    (car (send self :larm-controller))
    ))
  |#
  (:larm-controller
   ()
   (list
    (list
     (cons :controller-action "larm_controller/follow_joint_trajectory")
     (cons :controller-state  "/panda/panda_effort_joint_trajectory_controller/follow_joint_trajectory")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names
           (remove-if #'(lambda (jn) (substringp "finger" jn))
                      (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n))
                              (send-all (send robot :larm :joint-list) :name)))))))
  (:head-controller
   ()
   (list
    (list
     (cons :controller-action "head_controller/follow_joint_trajectory")
     (cons :controller-state "head_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n))
                                (send-all (send robot :head :joint-list) :name))))))
  (:lhand-controller
   ()
   (list
    (list
     (cons :controller-action "lhand_controller/follow_joint_trajectory")
     (cons :controller-state "lhand_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "left_finger_joint1" "left_finger_joint2") ))))
  #|
  (:start-grasp
  (arm &key (wait nil)) ;; TODO :arms is not implemented
  (unless (memq arm '(:larm))
  (error "you must specify arm ~A from ~A" (car args) '(:larm))
  (return-from :start-grasp nil))
  (send self :move-gripper arm -0.01 :effort 1.0 :wait wait)
  )
  (:stop-grasp
  (arm &key (wait nil)) ;; TODO :arms is not implemented
  (unless (memq arm '(:larm))
  (error "you must specify arm ~A from ~A" (car args) '(:larm))
  (return-from :stop-grasp nil))
  (send self :move-gripper arm 0.05 :effort 1.0 :wait wait)
  )
  (:get-table nil controller-table)
  (:move-gripper
  (arm pos &key (effort 25) (tm 500) (timeout 5000) (wait nil))  ;; TODO :arms is not implemented
  (let* ((start-time 0.1) ;; TODO fixed start-time
  (action (case arm
  (:larm gripper-action)))
  (joint-names (case arm
  (:larm (cdr (assoc :joint-names (car (send self :lhand-controller)))))))
  (goal (send action :make-goal-instance))
  (st (if (numberp start-time)
  (ros::time+ (ros::time-now) (ros::time start-time))
  start-time)))
  ;; send command
  (send goal :header :seq 1)
  (send goal :header :stamp st)
  (send goal :goal :trajectory :joint_names joint-names)
  (send goal :goal :trajectory :header :stamp st)
  (send goal :goal :trajectory :points
  (list (instance trajectory_msgs::JointTrajectoryPoint
  :init
  :positions (float-vector pos pos)
  :effort (float-vector effort effort)   ;; effort control
  :time_from_start (ros::time (/ tm 1000.0)))))
  (send self :spin-once)
  (send action :send-goal goal)
  (cond (wait (send action :wait-for-result :timeout timeout))))) ;; wait if needed
  (:gripper
  (&rest args)
  (when (eq (car args) :arms)
  (return-from :gripper
  (mapcar #'(lambda (x)
  (send self :gripper x (cadr args)))
  '(:larm)))
  (unless (memq (car args) '(:larm))
  (error "you must specify arm ~A from ~A" (car args) '(:larm))
  (return-from :gripper nil))
  (send self :state
  (intern
  (format nil "~A-~A" (string (car args)) (string (cadr args)))
  *keyword-package*)))
  (:gripper-state-callback
  (arm msg)
  (let* ((poss (send (send msg :actual) :positions))
  (vels (send (send msg :actual) :velocities)))
  (dolist (i (list
  (cons (format nil "~A-POSITION" (string arm))
  (* 1000.0 (/ (+ (elt poss 0) (elt poss 1)) 2)))
  (cons (format nil "~A-VELOCITY" (string arm))
  (* 1000.0 (/ (+ (elt vels 0) (elt vels 1)) 2)))))
  (send self :set-robot-state1 (intern (car i) *keyword-package*) (cdr i)))))
  |#
  ;;;;
  (:set-joint-pd-gain
   (joint-name pgain dgain)
   (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
     (send req :config :doubles
           (list (instance dynamic_reconfigure::DoubleParameter :init
                           :name "p" :value pgain)
                 (instance dynamic_reconfigure::DoubleParameter :init
                           :name "d" :value dgain)))
     (ros::service-call
      (format nil "/panda/panda_effort_joint_trajectory_controller/gains/~A/set_parameters" joint-name)
      req)
     ))
  (:set-all-joint-pd-gain
   (pgain dgain)
   (dolist (j (send robot :joint-list))
     (send self :set-joint-pd-gain (send j :name) pgain dgain))
   )
  (:check-error ()
                (ros::spin-once "error_group")
                (or error)
                )
  (:callback-larm-error (msg)
                        (setq error (send msg :data))
                        )
  (:wait-recover-error () (send error-recovery-act :wait-for-result))
  (:recover-error (&key (wait t))
                  (let ((goal (instance franka_msgs::ErrorRecoveryActionGoal :init)))
                    (send goal :header :stamp (ros::time-now))
                    (send error-recovery-act :send-goal goal)
                    (if wait (send self :wait-recover-error))
                    ))
  ;; gripper action for real-controller
  (:send-gripper-grasp-action
   (act width speed force &key (wait t) (inner 0.005) (outer 0.005))
   (let ((goal (instance franka_gripper::GraspActionGoal :init)))
     (send goal :header :stamp (ros::time-now))
     (send goal :goal :width width) ;; [m]
     (send goal :goal :speed speed) ;; [m/s]
     (send goal :goal :force force) ;; [N]
     (send goal :goal :epsilon :inner inner) ;; [m]
     (send goal :goal :epsilon :outer outer) ;; [m]
     ;;
     (send act :send-goal goal)
     (if wait (send act :wait-for-result))
     ))
  (:send-gripper-homing-action
   (act &key (wait t))
   (let ((goal (instance franka_gripper::HomingActionGoal :init)))
     (send goal :header :stamp (ros::time-now))
     ;;
     (send act :send-goal goal)
     (if wait (send act :wait-for-result))
     ))
  (:send-gripper-move-action
   (act width speed &key (wait t))
   (let ((goal (instance franka_gripper::MoveActionGoal :init)))
     (send goal :header :stamp (ros::time-now))
     (send goal :goal :width width) ;; [m]
     (send goal :goal :speed speed) ;; [m/s]
     ;;
     (send act :send-goal goal)
     (if wait (send act :wait-for-result))
     ))
  (:send-gripper-stop-action
   (act &key (wait t))
   (let ((goal (instance franka_gripper::StopActionGoal :init)))
     (send goal :header :stamp (ros::time-now))
     ;;
     (send act :send-goal goal)
     (if wait (send act :wait-for-result))
     ))
  (:stop-gripper
   (arm &key (wait nil))
   (let (acts)
     (case
         arm
       (:larm (setq acts (list gripper-stop-action)))
       )
     (when acts
       (dolist (act acts)
         (send self :send-gripper-stop-action act
               pos (/ (* 1000 0.08) tm) :wait nil))
       (if wait (mapcar #'(lambda (act) (send act :wait-for-result)) acts))
       )))
  (:homing-gripper
   (arm &key (wait nil))
   (let (acts)
     (case
         arm
       (:larm (setq acts (list gripper-homing-action)))
       )
     (when acts
       (dolist (act acts)
         (send self :send-gripper-homing-action act
               pos (/ (* 1000 0.08) tm) :wait nil))
       (if wait (mapcar #'(lambda (act) (send act :wait-for-result)) acts))
       )))
  (:start-grasp
   (arm &key (effort 80.0) (tm 500) (wait nil)) ;; TODO :arms is not implemented
   (let (acts)
     (case
         arm
       (:larm (setq acts (list gripper-grasp-action)))
       )
     (when acts
       (dolist (act acts)
         (send self :send-gripper-grasp-action act
               0.0 (/ (* 1000 0.08) tm) effort :wait nil))
       (if wait (mapcar #'(lambda (act) (send act :wait-for-result)) acts))
       )))
  (:stop-grasp
   (arm &key (wait nil)) ;; TODO :arms is not implemented
   (unless (memq arm '(:larm :arms))
     (error "you must specify arm ~A from ~A" (car args) '(:larm :arms))
     (return-from :stop-grasp nil))
   (send self :move-gripper arm 0.08 :tm 500 :wait wait)
   )
  (:move-gripper
   (arm pos &key (effort 25) (tm 500) (timeout 5000) (wait nil))  ;; TODO :arms is not implemented
   (let (acts)
     (case
         arm
       (:larm (setq acts (list gripper-move-action)))
       )
     (when acts
       (dolist (act acts)
         (send self :send-gripper-move-action act
               pos (/ (* 1000 0.08) tm) :wait nil))
       (if wait (mapcar #'(lambda (act) (send act :wait-for-result)) acts))
       )))
  )

;; grasp controller ...
(defun panda-init ()
  (setq *ri* (instance panda-robot-interface :init))
  (setq *robot* (panda))
  )

#|
(send *ri* :set-all-joint-pd-gain 1000.0 5.0) ;; default
(send *ri* :set-all-joint-pd-gain  300.0 5.0) ;; hard
(send *ri* :set-all-joint-pd-gain   30.0 0.5) ;; soft
|#
