<?php
// --------------------------------------------------------------------------
//
//  Copyright (C) 2015 Dennis Stampfer, Matthias Lutz and others
//
//        Servicerobotik Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// --------------------------------------------------------------------------

include('/tmp/SmartFionaNavigator.out.txt');
include('/tmp/SmartFionaNavigator-instructions.out.txt');

echo "Your Location: ".$symbolicLocation."<p>";

echo "<p>The following points of interest are within 2m: ";
echo join(", ", $lbs);
echo "<p>";

if(file_exists($symbolicLocation.".jpg")) {
	echo "Additional information about this exhibit (".$symbolicLocation."):<p><img src='".$symbolicLocation.".jpg' style='width: 100%;' ".(getimagesize($symbolicLocation.".jpg")[3]).">";
} else {
	echo "No information about this exhibit available (".$symbolicLocation.").";
}
?>

