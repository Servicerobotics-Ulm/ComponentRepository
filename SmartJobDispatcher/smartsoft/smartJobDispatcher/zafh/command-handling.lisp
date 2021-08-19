;;--------------------------------------------------------------------------
;  BSD 3-Clause License
;
;  Copyright (C) Servicerobotics Ulm
;  University of Applied Sciences Ulm
;  Prittwitzstr. 10
;  89075 Ulm
;  Germany
;  All rights reserved.
;
;  Author: Matthias Lutz, Matthias Rollenhagen
;
;Redistribution and use in source and binary forms, with or without
;modification, are permitted provided that the following conditions are met:
;
;* Redistributions of source code must retain the above copyright notice, this
;  list of conditions and the following disclaimer.
;
;* Redistributions in binary form must reproduce the above copyright notice,
;  this list of conditions and the following disclaimer in the documentation
;  and/or other materials provided with the distribution.
;
;* Neither the name of the copyright holder nor the names of its
;  contributors may be used to endorse or promote products derived from
;  this software without specific prior written permission.
;
;THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
;FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
;DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
;SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
;OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;;--------------------------------------------------------------------------

(defun handle-command (request)
  "This dunction handles the command requests"

  (let* ((command (cdr (assoc 'command request)))
         (command-type (cdr (assoc 'type command))))

    (format t "[handleCommand] found command: ~s~%" command)
    (format t "[handleCommand] found command type: ~s~%" command-type)

    (cond
          ((string-equal command-type "replace-positions")
            (format t "replace-positions found~%")
            (cond
              ((equal (cdr (assoc 'positions command)) nil)
                (format t "Postion list is emtpy --> delete all positions~%")
                (tcl-kb-delete :key '(is-a) :value `((is-a location))))
              (T
                (let ((kb-update-list nil))
                  (dolist (pose (cdr (assoc 'positions command)))
                    (multiple-value-bind (id x y yaw type approach-type) (values (cdr (assoc 'position-id pose)) (cdr (assoc 'x pose)) 
                    (cdr (assoc 'y pose)) (cdr (assoc 'phi pose)) (string-downcase (string (cdr (assoc 'type pose)))) (cdr (assoc 'approach-type pose))  )
                      (format t "Position ID: ~s  x: ~s  y: ~s phi: ~s type:~s ~%" id x y yaw type)
                      (push (list '(is-a name)
                           `( 
                            (is-a location)
                            (name ,id)
                            (type ,(read-from-string type))
                            ;(approach-type (path-nav))
                            (approach-type ,(read-from-string approach-type))
                            (approach-region-pose (,(round(* x 1000)),(round (* y 1000)) 0))
                            (approach-region-dist 75)
                            (approach-exact-pose (,(round (* x 1000)),(round (* y 1000)) 0))
                            (approach-exact-dist 100)
                            (approach-exact-safetycl 0)
                            (orientation-region (angle-absolute ,(round yaw)))
                            (orientation-exact (angle-absolute ,(round yaw)))
                            ;(backward-dist ,(sixth request))
                            ;(backward-rotation ,(seventh request))
                            (parking-state free))) kb-update-list)))

                  (tcl-kb-update-batch :updates-list kb-update-list :delete-key '(is-a) :delete-value '((is-a location))))))

                  (format t "=== locations-list ===~%")

		  (let ((obj-list     (tcl-kb-query-all :key '(is-a) :value '((is-a location)))))
		    (dolist (obj obj-list)
		      (format t "~%--------------------------------~%")
		      (format t "name                       : ~s ~%" (get-value obj 'name))
		      (format t "  type                     : ~s ~%" (get-value obj 'type))
		      (format t "  approach-type            : ~s ~%" (get-value obj 'approach-type))
		      (format t "  approach-region-pose     : ~s ~%" (get-value obj 'approach-region-pose))
		      (format t "  approach-region-dist     : ~s ~%" (get-value obj 'approach-region-dist))
		      (format t "  approach-exact-pose      : ~s ~%" (get-value obj 'approach-exact-pose))
		      (format t "  approach-exact-dist      : ~s ~%" (get-value obj 'approach-exact-dist))
		      (format t "  approach-exact-safetycl  : ~s ~%" (get-value obj 'approach-exact-safetycl))
		      (format t "  orientation-region       : ~s ~%" (get-value obj 'orientation-region))
		      (format t "  orientation-exact        : ~s ~%" (get-value obj 'orientation-exact))
		      (format t "  orientation              : ~s ~%" (get-value obj 'orientation))
		      (format t "  station-distance         : ~s ~%" (get-value obj 'station-distance))
		      (format t "  belt-count               : ~s ~%" (get-value obj 'belt-count))))

            (save-positions-from-kb-to-file)
            ;;[Timo]
            ;(send-command-to-all-robots (encode-msg '((msg-type . "robot-msg") (msg . ((type . "clear-robots-current-symbolic-position"))))))
            (send-command-to-all-robots '(ClearRobotsCurrentSymbolicPosition))
            
      )
      
      ((string-equal command-type "replace-stations")
	    
	    (cond
	      ((equal (cdr (assoc 'stations command)) nil)
		(format t "Stations list is emtpy --> delete all stations~%")
		(tcl-kb-delete :key '(is-a) :value `((is-a station))))
	      (T
		(let ((kb-update-list nil))
		  (dolist (station (cdr (assoc 'stations command)))
		    (multiple-value-bind (id x y yaw numBelts type docking-type approach-location) (values (cdr (assoc 
		    'station-id station)) (cdr (assoc 'x station)) (cdr (assoc 'y station)) (cdr (assoc 'phi station)) 
		     (cdr (assoc 'num-belts station)) (string-downcase (string (cdr (assoc 'station-type station)))) 
		     (string-downcase (string (cdr (assoc 'docking-type station)))) (cdr (assoc 'approach-location station)) )
		      (format t "Station ID: ~s  x: ~s  y: ~s phi: ~s numBelts: ~s type: ~s docking-type: ~s 
		      app.Loc: ~s ~%" id x y yaw numBelts type docking-type approach-location)
		      (let ( (station-pose `(,x ,y 0.0 ,(* (/ yaw 180) pi) 0.0 0.0)))
		        (push (list '(is-a id)
		         `( 
		           (is-a station)
		           (id ,id)
		           (type ,(read-from-string type))
		           (belt-count ,numBelts)
		           (docking-type ,(read-from-string docking-type)) ; LASER-IR
		           (approach-location, approach-location)
		           (pose ,station-pose)))
		           kb-update-list))
		     )
		  )

		 (tcl-kb-update-batch :updates-list kb-update-list :delete-key '(is-a) :delete-value '((is-a station))))
	      )
	    )

	    (format t "ReplaceStations bevor write!~%")
	    (show-stations)

	    (save-stations-from-kb-to-file)
       )
       
       ((string-equal command-type "replace-shelfs")
	(format t "===> 1 ~%")
            (cond
              ((equal (cdr (assoc 'shelfs command)) nil)
                (format t "Shelfs list is emtpy --> delete all shelfs~%")
                (tcl-kb-delete :key '(is-a) :value `((is-a rack))))
              (T
                (let ((kb-update-list nil))
		  (format t "===> 2 ~%")
                  (dolist (shelf (cdr (assoc 'shelfs command)))
		    (format t "===> 3 ~%")
                    (multiple-value-bind (id shelf-type pre-grasp-pose pull-out-pose obj-recog-ptu-poses 
                    planned-grasping location-id recognition-component post-grasping-state dimensions pick-distance levels)
                    (values (cdr (assoc 'shelf-id shelf)) (cdr (assoc 'type shelf)) (cdr (assoc 'pre-grasp-pose shelf)) 
                    (cdr (assoc 'pull-out-pose shelf)) (cdr (assoc 'obj-recog-ptu-poses shelf)) 
                    (cdr (assoc 'planned-grasping shelf)) (cdr (assoc 'location-id shelf))  
                    (cdr (assoc 'recognition-component shelf)) (cdr (assoc 'post-grasping-state shelf)) 
                    (cdr (assoc 'dimensions shelf)) (cdr (assoc 'pick-distance shelf)) (cdr (assoc 'levels shelf))  )
                    
                    (let ((ptu-poses-list nil))
		      (format t "===> 4 ~%")                 
                       (dolist (ptu-pose obj-recog-ptu-poses)
			 (format t "===> 5 ~%")
                         (push `( ,(cdr (assoc 'pan ptu-pose)) ,(cdr (assoc 'tilt ptu-pose)) )
                             ptu-poses-list)

                         (format t "ptu pose ~a ~%" (list `( ,(cdr (assoc 'pan ptu-pose)) ,(cdr (assoc 'tilt ptu-pose)) )) )
                       )
                      
                       (push (list '(is-a id)
                         `( 
                           (is-a rack)
                           (id ,(read-from-string id))
                           (type ,(read-from-string shelf-type))
                           (pre-grasp-pose ( ,(cdr (assoc 'x pre-grasp-pose)) ,(cdr (assoc 'y pre-grasp-pose)) 
                           ,(cdr (assoc 'z pre-grasp-pose)) ,(cdr (assoc 'yaw pre-grasp-pose)) 
                           ,(cdr (assoc 'pitch pre-grasp-pose)) ,(cdr (assoc 'roll pre-grasp-pose))) )
                           (pull-out-pose ( ,(cdr (assoc 'x pull-out-pose)) ,(cdr (assoc 'y pull-out-pose)) 
                           ,(cdr (assoc 'z pull-out-pose)) ,(cdr (assoc 'yaw pull-out-pose)) 
                           ,(cdr (assoc 'pitch pull-out-pose)) ,(cdr (assoc 'roll pull-out-pose))))
                           (obj-recog-ptu-poses ,(reverse ptu-poses-list))
                           (planned-grasping ,planned-grasping)
                           (location-id ,location-id)
                           (recognition-component ,(string recognition-component))
                           (post-grasping-state ,(read-from-string post-grasping-state))
                           (dimensions ( ,(cdr (assoc 'width dimensions)) ,(cdr (assoc 'depth dimensions)) 
                           ,(cdr (assoc 'height dimensions))) )
                           (levels ,levels)
                           (pick-distance ( ,(cdr (assoc 'x pick-distance)) ,(cdr (assoc 'y pick-distance)) ) )))
                         kb-update-list)

                      (format t "shelf KB update list: ~a  ~%" kb-update-list)
                      (tcl-kb-update-batch :updates-list kb-update-list :delete-key '(is-a) :delete-value '((is-a rack)))
                    )
                   )
                  )
                )
              )
             )
                  
             (format t "=== shelf-list ===~%")
             (let ((shelf-list (tcl-kb-query-all :key '(is-a) :value '((is-a rack)))))
             	(dolist (shelf shelf-list)
		     (format t "~%--------------------------------~%")
		     (format t "id                    : ~a ~%" (get-value shelf 'id))
		     (format t "type                  : ~a ~%" (get-value shelf 'type))
		     (format t "pre-grasp-pose        : ~a ~%" (get-value shelf 'pre-grasp-pose))
		     (format t "pull-out-pose         : ~a ~%" (get-value shelf 'pull-out-pose))
		     (format t "obj-recog-ptu-poses   : ~a ~%" (get-value shelf 'obj-recog-ptu-poses))
		     (format t "planned-grasping      : ~a ~%" (get-value shelf 'planned-grasping))
		     (format t "location-id           : ~a ~%" (get-value shelf 'location-id))
		     (format t "recognition-component : ~a ~%" (get-value shelf 'recognition-component))
		     (format t "post-grasping-state   : ~a ~%" (get-value shelf 'post-grasping-state))
		     (format t "dimensions            : ~a ~%" (get-value shelf 'dimensions))
		     (format t "pick-distance         : ~a ~%" (get-value shelf 'pick-distance))
		     (format t "levels                : ~a ~%" (get-value shelf 'levels))
             	)
             )

            (format t "replace-shelfs done! ~%")
       )
       
       ((string-equal command-type "replace-items")
	(format t "===> 1 ~%")
         (cond
              ((equal (cdr (assoc 'items command)) nil)
                (format t "Item list is emtpy --> delete all items~%")
                (tcl-kb-delete :key '(is-a) :value `((is-a object-class))))
              (T
                (let ((kb-update-list nil)
                      (ptu-poses-list nil))
		  (format t "===> 2 ~%")
                  (dolist (item (cdr (assoc 'items command)))
		    (format t "===> 3 ~%")
                    (multiple-value-bind (item-type rack-id shelf-level slot) (values (cdr (assoc 'type item)) 
                    (cdr (assoc 'rack-id item)) (cdr (assoc 'shelf-level item)) (cdr (assoc 'slot item)) )
     
		    (format t "===> 4 ~%")    

                        (push (list '(is-a type)
                         `( 
                           (is-a object-class)
                           (type ,(read-from-string item-type))
                           (rack-id ,(read-from-string rack-id))
                           (shelf-level ,shelf-level)
                           (slot ,slot)  ))
                         kb-update-list)

		         (format t "Item KB update list: ~a  ~%" kb-update-list)
		         (tcl-kb-update-batch :updates-list kb-update-list :delete-key '(is-a) :delete-value 
		         '((is-a object-class)))
                    )
                  )
                )
              )
        )
                  
        (format t "=== item-list ===~%")
        (let ((item-list (tcl-kb-query-all :key '(is-a) :value '((is-a object-class)))))
	  (dolist (item item-list)
	  (format t "~%--------------------------------~%")
	  (format t "type        : ~a ~%" (get-value item 'type))
	  (format t "rack-id     : ~a ~%" (get-value item 'rack-id))
	  (format t "shelf-level : ~a ~%" (get-value item 'shelf-level))
	  (format t "slot        : ~a ~%" (get-value item 'slot))
	  )
        )
         (format t "replace-item done! ~%")
       )
            
      (T
        (format t "Error not supported COMMAND!~%")))))
