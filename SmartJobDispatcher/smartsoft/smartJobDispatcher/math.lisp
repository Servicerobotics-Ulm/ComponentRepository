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
;  Author: Matthias Lutz
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




;; (transformPoseToPoint 0.4055 0 1.0065 0.1651 0.0087 0.0094 -0.007602 0.0070168 0.06254)
;; (transformPoseToPoint 0.4378 -0.0588 1.0049 0.0757 0.0297 0.014 -0.007602 0.0070168 0.06254)
;; (transformPoseToPoint 1.0 0.0 0.0 PI 0.0 0.0 1.0 0.0 0.0)
(defun transformPoseToPoint (x y z yaw pitch roll xt yt zt)
  (let ((xp nil)(yp nil)(zp nil))
    (setf xp
      (- (+ x
            (* zt
               (+ (* (sin roll) (sin yaw))
                  (* (cos roll) (cos yaw) (sin pitch))))
            (* xt
               (* (cos pitch) (cos yaw))))
         (* yt
            (- (* (cos roll) (sin yaw))
               (* (cos yaw) (sin pitch) (sin roll))))))
    (setf yp
      (- (+ y    
            (* yt
               (+ (* (cos roll) (cos yaw))
                  (* (sin pitch) (sin roll) (sin yaw))))
            (* xt
               (* (cos pitch) (sin yaw))))
          (* zt
             (- (* (cos yaw) (sin roll))
                (* (sin pitch) (cos roll) (sin yaw))))))
    (setf zp
      (- (+ z
            (* zt
               (* (cos pitch) (cos roll)))
            (* yt
               (* (cos pitch) (sin roll))))
         (* xt
            (sin pitch))))
    `(,xp ,yp ,zp)))


;; (getPointRelativeToPose 0.418677 -0.167277 0.690096 (- 1.179696 (/ pi 2)) (+ 1.518436 (/ pi 2)) 0.0 0.407583 -0.16270301 0.680097) 
(defun getPointRelativeToPose (x y z yaw pitch roll xt yt zt)
  (let ((xp nil)(yp nil)(zp nil))
    (setf xp
      (- (+
           (* z (sin pitch))
           (* xt (cos pitch) (cos yaw))
           (* yt (cos pitch) (sin yaw)))
         (* zt (sin pitch))
         (* x (cos pitch) (cos yaw))
         (* y (cos pitch) (sin yaw))))
    (setf yp
      (- (+
           (* yt (cos roll) (cos yaw))
           (* zt (cos pitch) (sin roll))
           (* x (cos roll) (sin yaw))
           (* xt (cos yaw) (sin pitch) (sin roll))
           (* yt (sin pitch) (sin roll) (sin yaw)))
         (* y (cos roll) (cos yaw))
         (* z (cos pitch) (sin roll))
         (* xt (cos roll) (sin yaw))
         (* x (cos yaw) (sin pitch) (sin roll))
         (* y (sin pitch) (sin roll) (sin yaw))))
    (setf zp
      (- (+
           (* zt (cos pitch) (cos roll))
           (* y (cos yaw) (sin roll))
           (* xt (sin roll) (sin yaw))
           (* xt (cos roll) (cos yaw) (sin pitch))
           (* yt (cos roll) (sin pitch) (sin yaw)))
         (* z (cos pitch) (cos roll))
         (* yt (cos yaw) (sin roll))
         (* x (sin roll) (sin yaw))
         (* x (cos roll) (cos yaw) (sin pitch))
         (* y (cos roll) (sin pitch) (sin yaw))))
    `(,xp ,yp ,zp))) 
      




;  (inverseComposeFrom2D 1.0 0.0 pi 0.0 1.0)
(defun inverseComposeFrom2D (pose_x pose_y pose_phi point_x point_y)
  "transform the point into the coordinatate frame of the pose"
  (format t "Pose x:~s y:~s phi:~s~%" pose_x pose_y pose_phi)
  (format t "Point x:~s y:~s~%" point_x point_y)

  (let ((resx nil) (resy nil)
        (ccos (cos pose_phi))
        (ssin (sin pose_phi)))
    (setf resx  (+ (* (- point_x pose_x) ccos) (* (- point_y pose_y) ssin)))
    (setf resy  (+ (* (* -1 (- point_x pose_x)) ssin) (* (- point_y pose_y) ccos)))
    (format t "Res x:~s y:~s~%" resx resy)
    (list resx resy)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; buildHomogeneousMatrixFromPose
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun buildHomogeneousMatrixFromPose (pose &key (result (make-array '(4 4))))
  "construct a homogeneous transformation matrix 4x4 from x y z yaw pitch roll"
  (multiple-value-bind (x y z yaw pitch roll)
    (apply #'values pose)
    ;(format t "Pose: x:~a y:~a z:~a yaw:~a pitch:~a roll:~a ~%" x y z yaw pitch roll)
    (setf (aref result 0 3) x)
    (setf (aref result 1 3) y)
    (setf (aref result 2 3) z)

    (setf (aref result 3 0) 0.0)
    (setf (aref result 3 1) 0.0)
    (setf (aref result 3 2) 0.0)
    (setf (aref result 3 3) 1.0)

    (let ((cy (cos yaw)) (sy (sin yaw)) (cp (cos pitch)) (sp (sin pitch)) (cr (cos roll)) (sr (sin roll)))
      (setf (aref result 0 0) (* cy cp))     (setf (aref result 0 1) (- (* cy sp sr) (* sy cr)))      (setf (aref result 0 2) (+ (* cy sp cr) (* sy sr)))
      (setf (aref result 1 0) (* sy cp))     (setf (aref result 1 1) (+ (* sy sp sr) (* cy cr)))      (setf (aref result 1 2) (- (* sy sp cr) (* cy sr)))
      (setf (aref result 2 0) (* -1.0 sp))   (setf (aref result 2 1) (* cp sr))                       (setf (aref result 2 2) (* cp cr))))
    result)

;  (print-rows-of-nxm-matrix (buildHomogeneousMatrixFromPose `(2.16855 -0.00828913  0.67 ,(* -1.0 (+ (/ pi 2) (/ pi 4))) 0  ,(/ pi 2))))
;  proven result:
;   -0.707107  4.32978e-17    -0.707107      2.16855
;   -0.707107 -4.32978e-17     0.707107  -0.00828913
;          -0            1  6.12323e-17         0.67
;           0            0            0            1
;  (print-rows-of-nxm-matrix (buildHomogeneousMatrixFromPose '(1.0 2.0 3.0 0.0 0.0 0.0)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; hypot
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun hypot (x y)
  (sqrt (+ (* x x) (* y y))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; convertHMtoPose
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun convertHMtoPose (HM &key (result '()))
  "convert HomogeneousMatrix to pose with euler angles"
  (let ((x nil) (y nil) (z nil) (yaw nil) (pitch nil) (roll nil))

    (setf pitch (atan (* -1.0 (aref HM 2 0)) (hypot (aref HM 0 0) (aref HM 1 0) )))

    (cond
      ((<  (+ (abs (aref HM 2 1)) (abs (aref HM 2 2))) (* 10 DOUBLE-FLOAT-EPSILON))
        ;;Gimbal lock case
        (setf roll 0.0)
        (if (> pitch 0)
          (setf yaw (atan (aref HM 1 2) (aref HM 0 2)))
          (setf yaw (atan (* -1.0 (aref HM 1 2)) (* -1.9 (aref HM 0 2))))))
      (T 
        (setf roll (atan (aref HM 2 1) (aref HM 2 2)))
        (setf yaw  (atan (aref HM 1 0) (aref HM 0 0)))))

    (setf x (aref HM 0 3))
    (setf y (aref HM 1 3))
    (setf z (aref HM 2 3))

    (setf result `(,x ,y ,z ,yaw ,pitch ,roll)))
  result)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; composePoses3D
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun composePoses3D (poseA poseB &key (result '()))
  "coordinate system transformation, compose pose a with pose b"

    (let ( (res_HM (make-array '(4 4)))
           (a_HM  (buildHomogeneousMatrixFromPose poseA))
           (b_HM  (buildHomogeneousMatrixFromPose poseB)))
      (setf res_HM (multiply-two-matrices a_HM b_HM))
      ;(format t "Res_HM: ~%")
      ;(print-rows-of-nxm-matrix res_HM)
      (setf result (convertHMtoPose res_HM)))
  result)

; (composePoses3D `(2.16855 -0.00828913  0.67 ,(* -1.0 (+ (/ pi 2) (/ pi 4))) 0  ,(/ pi 2)) `(0.0 0.0 0.0 ,(/ pi 2) ,(/ pi 2) 0.0))
; proven result: (2.1686,-0.0083,0.6700,-0.7853981,-0.00deg,0.00deg)

;(composePoses3D (composePoses3D `(2.16855 -0.00828913  0.67 ,(* -1.0 (+ (/ pi 2) (/ pi 4))) 0  ,(/ pi 2)) `(0.0 0.0 0.0 ,(/ pi 2) ,(/ pi 2) 0.0)) '(-0.5 0.0 0.0 0.0 0.0 0.0))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;; code below from http://faculty.washington.edu/dbp/SAPACLISP-1.x/matrix.lisp
;; Version 1.0; Copyright 1993, Donald B. Percival, All Rights Reserved

;;;-*- Mode: LISP; Package: :SAPA; Syntax: COMMON-LISP -*-
;-------------------------------------------------------------------------------
;-------------------------------------------------------------------------------
;
;  matrix.lisp
;
;  a collection of Lisp functions for some basic matrix operations ...
;  Note:  before compiling and loading matrix.lisp,
;         you should compile and load
;         sapa-package.lisp
;
;  6/29/95
;
;  SAPA, Version 1.0; Copyright 1993, Donald B. Percival, All Rights Reserved
;
;  Use and copying of this software and preparation of derivative works
;  based upon this software are permitted.  Any distribution of this
;  software or derivative works must comply with all applicable United
;  States export control laws.
; 
;  This software is made available AS IS, and no warranty -- about the
;  software, its performance, or its conformity to any
;  specification -- is given or implied. 
;
;  Comments about this software can be addressed to dbp@apl.washington.edu
;-------------------------------------------------------------------------------
;  6/29/95: corrected a minor typo in documentation for modified-Gram-Schmidt!
;           ("rank N" replaced by "rank n")
;  6/29/95: added extract-column and extract-row
;-------------------------------------------------------------------------------


;-------------------------------------------------------------------------------
;-------------------------------------------------------------------------------
;;;  The functions  print-1-or-2-d-array
;;;                 print-rows-of-nxm-matrix
;;;  can be used to print out the elements of one and two dimensional array.
;-------------------------------------------------------------------------------
;-------------------------------------------------------------------------------
(defun print-1-or-2-d-array
       (an-array
        &key
        (tag "array")
        (format-control-for-array-element "~F"))
  "given
   [1] an-array (required)
       ==> a one-dimensional or two-dimensional array
   [2] tag (keyword; `array')
       ==> an optional tag to be printed
           along with array elements
   [3] format-control-for-array-element (keyword; `~F')
       ==> format control for a single array element
prints the elements of an-array and
returns
   [1] an-array"
  (let ((dims (array-dimensions an-array))
        (for-format (concatenate 'string
                                 "~&~A, element ~A: "
                                 format-control-for-array-element)))
    (cond
     ((= (length dims) 1)
      ;;; a vector ...
      (dotimes (i (car dims) (values an-array))
        (format t for-format tag i (aref an-array i))))
     ((= (length dims) 2)
      (dotimes (i (nth 0 dims) (values an-array))
        (dotimes (j (nth 1 dims))
          (format t for-format tag (list i j) (aref an-array i j)))))
     (t
      (error "can't print ~A" an-array)))))


;(print-1-or-2-d-array #(1 2 3 4))
;;==>
;array, element 0: 1.0
;array, element 1: 2.0
;array, element 2: 3.0
;array, element 3: 4.0
;#(1 2 3 4)
;
;  (print-1-or-2-d-array (make-array '(3 2) :initial-contents '((1 2) (3 4) (5 6))))
;==>
;array, element (0 0): 1.0
;array, element (0 1): 2.0
;array, element (1 0): 3.0
;array, element (1 1): 4.0
;array, element (2 0): 5.0
;array, element (2 1): 6.0
;#2a((1 2) (3 4) (5 6))

;-------------------------------------------------------------------------------
(defun print-rows-of-nxm-matrix
       (an-nxm-matrix
        &key
        (format-control-for-array-element "~F "))
  "given
   [1] an-nxm-matrix (required)
       ==> a two-dimensional array
   [3] format-control-for-array-element (keyword; `~F')
       ==> format control for a single array element
prints the elements of the 2d array and
returns
   [1] an-nxm-matrix"
  (let ((dimensions (array-dimensions an-nxm-matrix)))
    (assert (= (length dimensions) 2))
    (let ((m-columns (elt dimensions 1)))
      (dotimes (i-row (elt dimensions 0))
        (format t "~&")
        (dotimes (j-col m-columns)
          (format t format-control-for-array-element
                  (aref an-nxm-matrix i-row j-col))))
      (format t "~&")
      (values an-nxm-matrix))))

; (print-rows-of-nxm-matrix (make-array '(3 2) :initial-contents '((1 2) (3 4) (5 6))))
;;==>
;1.0 2.0 
;3.0 4.0 
;5.0 6.0
;#2a((1 2) (3 4) (5 6))

;-------------------------------------------------------------------------------
;-------------------------------------------------------------------------------
;;;  The functions  transpose
;;;                 Hermitian-transpose
;;;                 zero-strict-lower-diagonal!
;;;                 multiply-two-matrices
;;;                 multiply-matrix-and-vector
;;;                 multiply-matrix-and-scalar
;;;                 subtract-two-matrices
;;;                 trace-matrix
;;;                 2d-matrix-move!
;;;  perform fairly simple operations on matrices and vectors.
;-------------------------------------------------------------------------------
;-------------------------------------------------------------------------------
(defun transpose
       (a-matrix
        &key
        (result
         (make-array
          (reverse (array-dimensions a-matrix)))))
  "given
   [1] A (required)
       ==> a 2d matrix
   [2] result (keyword; new 2d array of appropriate size)
       <== a 2d matrix to contain transpose of a-matrix
returns
   [1] transpose of a-matrix (placed in result)"
  (let ((list-of-two-integers (array-dimensions a-matrix)))
    (dotimes (i (nth 0 list-of-two-integers) result)
      (dotimes (j (nth 1 list-of-two-integers))
        (setf (aref result j i)
              (aref a-matrix i j))))))


; (transpose #2a((1 2) (3 4) (5 6)))
;;==> #2a((1 3 5) (2 4 6))

;-------------------------------------------------------------------------------
(defun Hermitian-transpose
       (a-matrix
        &key
        (result
         (make-array
          (reverse (array-dimensions a-matrix)))))
  "given
   [1] A (required)
       ==> a 2d matrix
   [2] result (keyword; new 2d array of appropriate size)
       <== a 2d matrix to contain Hermitian transpose of a-matrix
returns
   [1] Hermitian transpose of a-matrix (placed in result)"
  (let ((list-of-two-integers (array-dimensions a-matrix)))
    (dotimes (i (nth 0 list-of-two-integers) result)
      (dotimes (j (nth 1 list-of-two-integers))
        (setf (aref result j i)
              (conjugate (aref a-matrix i j)))))))

; (Hermitian-transpose #2a((1 2) (3 4) (5 6)))
; ;==> #2a((1 3 5) (2 4 6))
; (Hermitian-transpose #2a((1 #c(2 1)) (#c(3 -1) 1)))
;==>                 #2a((1 #c(3 1)) (#c(2 -1) 1))

;-------------------------------------------------------------------------------
(defun zero-strict-lower-diagonal!
       (a-matrix)
  "given a square matrix,
zeros its lower diagonal and returns
the modified square matrix"
  (let* ((n (nth 0 (array-dimensions a-matrix)))
         (n-in-column-to-zap (1- n))
         (row-start 1))
    (dotimes (j-column (1- n) a-matrix)
      (dotimes (i n-in-column-to-zap)
        (setf (aref a-matrix (+ i row-start) j-column ) 0.0))
      (incf row-start)
      (decf n-in-column-to-zap))))

;(zero-strict-lower-diagonal! #2a((1 #c(2 1)) (#c(3 -1) 1)))
;;==>                         #2a((1 #c(2 1)) (0.0 1))
;(zero-strict-lower-diagonal! #2a((1 2 3)   (4 5 6)     (7 8 9)))
;;==>                         #2a((1 2 3) (0.0 5 6) (0.0 0.0 9))


;-------------------------------------------------------------------------------
(defun multiply-two-matrices
       (a-matrix
        b-matrix
        &key
        (result
         (make-array
          (list (nth 0 (array-dimensions a-matrix))
                (nth 1 (array-dimensions b-matrix))))))
  "given
   [1] a-matrix (required)
       ==> a 2d matrix
   [2] b-matrix (required)
       ==> another 2d matrix, with dimensions such that
           the product of a-matrix and b-matrix is defined
   [3] result (keyword; new 2d array of appropriate size)
       <== a 2d matrix to contain product of two matrices
returns
   [1] product of two matrices (placed in result)"
  (let ((m (nth 0 (array-dimensions a-matrix)))
        (n (nth 1 (array-dimensions b-matrix)))
        (common (nth 0 (array-dimensions b-matrix))))
    (dotimes (i m result)
      (dotimes (j n)
        (setf (aref result i j) 0.0)
        (dotimes (k common)
          (incf (aref result i j)
                (* (aref a-matrix i k) (aref b-matrix k j))))))))


; (multiply-two-matrices #2a((0 0 1) (0 1 0) (1 0 0))
;                       #2a((10 9) (8 7) (6 5)))
;;==> #2a((6.0 5.0) (8.0 7.0) (10.0 9.0))

;-------------------------------------------------------------------------------
(defun multiply-matrix-and-vector
       (a-matrix
        b-vector
        &key
        (result
         (make-array
          (nth 0 (array-dimensions a-matrix)))))
  "given
   [1] a-matrix (required)
       ==> a 2d matrix
   [2] b-vector (required)
       ==> a vector, with dimensions such that
           the product of a-matrix and b-vector is defined
   [3] result (keyword; new vector of appropriate size)
       <== a vector to contain product of a-matrix and b-vector
returns
   [1] product of a-matrix and b-vector (placed in result)"
  (let ((m (nth 0 (array-dimensions a-matrix)))
        (n (length b-vector)))
    (dotimes (i m result)
      (setf (aref result i) 0.0)
      (dotimes (j n)
        (incf (aref result i)
              (* (aref a-matrix i j) (aref b-vector j)))))))


; (multiply-matrix-and-vector #2a((0 0 1) (0 1 0) (1 0 0))
;                            #(10 9 8))
;;==> #(8.0 9.0 10.0)

;-------------------------------------------------------------------------------
(defun multiply-matrix-and-scalar
       (a-matrix
        scalar
        &key
        (result
         (make-array
          (array-dimensions a-matrix))))
  "given
   [1] a-matrix (required)
       ==> a 2d matrix
   [2] scalar (required)
       ==> an arbitrary number
   [3] result (keyword; new matrix of same size as a-matrix)
       <== a matrix to contain product of a-matrix and scalar
returns
   [1] product of a-matrix and scalar (placed in result)"
  (let ((m (nth 0 (array-dimensions a-matrix)))
        (n (nth 1 (array-dimensions a-matrix))))
    (dotimes (i m result)
      (dotimes (j n)
        (setf (aref result i j)
              (* scalar (aref a-matrix i j)))))))


; (multiply-matrix-and-scalar #2a((0 0 1) (0 1 0) (1 0 0))
;                            1/3)
;;==> #2a((0 0 1/3) (0 1/3 0) (1/3 0 0))

;-------------------------------------------------------------------------------
(defun subtract-two-matrices
       (a-matrix
        b-matrix
        &key
        (result
         (make-array (array-dimensions a-matrix))))
  "given
   [1] a-matrix (required)
       ==> a 2d matrix
   [2] b-matrix (required)
       ==> a 2d matrix, with dimensions the same
           as a-matrix
   [3] result (keyword; new vector of appropriate size)
       <== a matrix to contain result of subtracting
           b-matrix from a-matrix
returns
   [1] a-matrix minus b-matrix (placed in result)"
  (let ((m (nth 0 (array-dimensions a-matrix)))
        (n (nth 1 (array-dimensions a-matrix))))
    (dotimes (i m result)
      (dotimes (j n)
        (setf (aref result i j)
              (- (aref a-matrix i j) (aref b-matrix i j)))))))


;(subtract-two-matrices #2a((1 0 0) (0 1 0) (0 0 1))
;                       #2a((0 0 1) (0 1 0) (1 0 0)))
;;==> #2a((1 0 -1) (0 0 0) (-1 0 1))

;-------------------------------------------------------------------------------
(defun trace-matrix (a-matrix)
  "given a square matrix,
returns its trace"
  (let ((m (nth 0 (array-dimensions a-matrix)))
        (n (nth 1 (array-dimensions a-matrix))))
    (assert (= m n))
    (let ((sum 0.0))
      (dotimes (i m sum)
        (incf sum (aref a-matrix i i))))))

;;; (trace-matrix #2a((1 2 3) (4 5 6) (7 8 9)))  ;==> 15.0

;-------------------------------------------------------------------------------
(defun 2d-matrix-move!
       (from-this
        to-this)
  "given
   [1] from-this (required)
       ==> a 2d matrix
   [2] to-this (required)
       <== another 2d matrix
transfer contents of from-this to corresponding
locations in to-this, and
returns
   [1] to-this"
  (let* ((temp (array-dimensions from-this))
         (n-rows (nth 0 temp))
         (n-columns (nth 1 temp)))
    (dotimes (i n-rows to-this)
      (dotimes (j n-columns)
        (setf (aref to-this i j) (aref from-this i j))))))

;(2d-matrix-move! #2a((1 2 3) (4 5 6) (7 8 9)) (make-array '(3 3)))
;;==> #2a((1 2 3) (4 5 6) (7 8 9))
;(2d-matrix-move! #2a((1 2 3) (4 5 6) (7 8 9)) (make-array '(4 4)))
;;==> #2a((1 2 3 nil) (4 5 6 nil) (7 8 9 nil) (nil nil nil nil))

;-------------------------------------------------------------------------------
(defun extract-column (2d-matrix column-index)
  "given
   [1] 2d-matrix (required)
       ==> a 2d matrix
   [1] column-index (required)
       ==> index of column to be extracted
returns
   [1] vector containing values in column indexed by column-index"
  (let* ((n-rows (nth 0 (array-dimensions 2d-matrix)))
         (column-vector (make-array n-rows)))
    (dotimes (i n-rows column-vector)
      (setf (aref column-vector i) (aref 2d-matrix i column-index)))))

;(extract-column #2a((1 2 3) (4 5 6) (7 8 9)) 0);
;;==> #(1 4 7)
;(extract-column #2a((1 2 3) (4 5 6) (7 8 9)) 1)
;;==> #(2 5 8)
;(extract-column #2a((2 3) (5 6) (8 9)) 1)
;;==> #(3 6 9)

;-------------------------------------------------------------------------------
(defun extract-row (2d-matrix row-index)
  "given
   [1] 2d-matrix (required)
       ==> a 2d matrix
   [1] row-index (required)
       ==> index of row to be extracted
returns
   [1] vector containing values in row indexed by row-index"
  (let* ((n-columns (nth 1 (array-dimensions 2d-matrix)))
         (row-vector (make-array n-columns)))
    (dotimes (i n-columns row-vector)
      (setf (aref row-vector i) (aref 2d-matrix row-index i)))))


;(extract-row #2a((1 2 3) (4 5 6) (7 8 9)) 0)
;;==> #(1 2 3)
;(extract-row #2a((1 2 3) (4 5 6) (7 8 9)) 1)
;;==> #(4 5 6)
;(extract-row #2a((2 3) (5 6) (8 9)) 1)
;;==> #(5 6)

