;;--------------------------------------------------------------------------
;;
;;  Copyright (C) 2014 Matthias Lutz
;;
;;	lutz@hs-ulm.de
;;
;;      ZAFH Servicerobotic Ulm
;;      Christian Schlegel
;;      University of Applied Sciences
;;      Prittwitzstr. 10
;;      89075 Ulm
;;      Germany
;;
;;
;;  This library is free software; you can redistribute it and/or
;;  modify it under the terms of the GNU Lesser General Public
;;  License as published by the Free Software Foundation; either
;;  version 2.1 of the License, or (at your option) any later version.
;;
;;  This library is distributed in the hope that it will be useful,
;;  but WITHOUT ANY WARRANTY; without even the implied warranty of
;;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;;  Lesser General Public License for more details.
;;
;;  You should have received a copy of the GNU Lesser General Public
;;  License along with this library; if not, write to the Free Software
;;  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
;;
;;--------------------------------------------------------------------------


(let ((indexpos nil)
      (tcl-prefix nil)
      (lispinterface-prefix ""))
  (setf indexpos  (position "--tcl-prefix" sb-ext:*posix-argv* :test #'equal))
  (cond
   ((not (eql indexpos nil))
    (setf tcl-prefix (nth (+ 1 indexpos) sb-ext:*posix-argv*))))
  (setf indexpos  (position "--module-path" sb-ext:*posix-argv* :test #'equal))
  (cond
   ((not (eql indexpos nil))
    (setf module-path (nth (+ 1 indexpos) sb-ext:*posix-argv*))))
  (setf indexpos  (position "--lispinterface-prefix" sb-ext:*posix-argv* :test #'equal))
  (cond
   ((not (eql indexpos nil))
    (setf lispinterface-prefix (nth (+ 1 indexpos) sb-ext:*posix-argv*))))

  (format t "all: ~s~%" sb-ext:*posix-argv*)
  (format t "tcl-Prefix: ~a~%" tcl-prefix)
  (format t "module-path: ~a~%" module-path)
  (format t "lispinterface-prefix: ~a~%" lispinterface-prefix)

;; MENU

(defun menu()
  (let ((nmbr nil)(dirlist nil))
  (setf dirlist (concatenate 'list (directory (format nil "~aload-*.lisp" tcl-prefix))))
  (format t "~%~%================================~%")
  (format t "= S C E N A R I O - M E N U ====~%")
  (format t "================================~%")
  (let ((counter 0))
  (dolist (file dirlist)
    (format t "[~a]: ~a~%" counter (file-namestring file))
    (setf counter (+ counter 1))))
    (format t "================================~%")
    (format t "[99]: exit to REPL~%")
    (format t "[-1]: quit~%")
    (format t "================================~%")
    (format t "Select scenario: ~%")
    (setf nmbr (parse-integer (read-line nil)))
    (format t "your choice: [~d] ~%" nmbr)
    
    (cond
      ((equal nmbr 99)
        (format t "type (menu) to come back to menu ~%")
        (return-from menu ()))
      ((equal nmbr -1)
        (format t "bye bye ~%")
        (exit))
      (T 
        ;;check if there is already an ini file in commandline args
        (cond 
          ;;if there is, this ini file arg is forwarded to the component
          ((loop for arg in sb-ext:*posix-argv*
            thereis (search "--filename=" arg))
           (format t "Found ini file param in posix-args~%"))
          (T
           ;;if not, use the scenario ini file and add this one to the posix-argv
           (format t "NO ini file param found in posix-args --> set the scenario ini file~%")
           (let ((start nil) (inifilename (file-namestring (nth nmbr dirlist))))
             (setf start (search ".lisp" inifilename :from-end t))
             (setf inifilename (subseq inifilename 0 start))
             (setf inifilename (subseq inifilename (length "load-")))
             (setf inifilename (format nil "SmartLispServer.ini.~a" inifilename))
             (nconc sb-ext:*posix-argv* (list (format nil "--filename=~a~a" tcl-prefix inifilename))))))
        (format t "all: ~s~%" sb-ext:*posix-argv*)
        (format t "load scenario file: ~a~%" (nth nmbr dirlist))
        ;;load the selected scenario file
        (load (nth nmbr dirlist))))))

(menu))







