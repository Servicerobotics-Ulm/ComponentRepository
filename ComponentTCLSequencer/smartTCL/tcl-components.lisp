;;--------------------------------------------------------------------------
;;
;;  Copyright (C) 	2011 Andreas Steck
;;
;;		steck@hs-ulm.de
;;
;;      ZAFH Servicerobotic Ulm
;;      Christian Schlegel
;;		of Applied Sciences
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

(defun show-components ()
  (let ((component-list     (query-kb-all *MEMORY* '(is-a) '((is-a component)))))
    (dolist (component component-list)
      (format t "~%--------------------------------~%")
      (format t "name             : ~s ~%" (get-value component 'name))
      (format t "  state          : ~s ~%" (get-value component 'state))
      (let ((slot-list (get-value component 'slots)))
        (format t "  slots          : ~s ~%" slot-list)
        (dolist (slot slot-list)
          (format t "    ~s(~s) ~%" slot (get-value component slot)))))))