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

include('./skripts/functions.php');
include('./skripts/header.php');
$basepath = getBaseUrl(); 
if (auth()){?>
<body> 
    <div data-role="page" id="info">
	<?php 
	printHeader();
	?>
        <nav data-role="navbar">
            <ul>
		<?php 
                printMenu();
		?>
            </ul>
        </nav>
        <div data-role="content" data-theme="d">

How to use the values:
<p>
random: <span id="random" style="">[LOADING]</span>

<p>
<canvas id="canvas" width="100" height="100"></canvas>
<script>
	//window.addEventListener("load", drawArrow); 
	
	var logoImage = new Image(); 
	logoImage.src = 'arrow.png';
	var context = $('#canvas')[0].getContext('2d'); 

	function drawArrow() {
		// get angle from remote
		//$.get("degrees.txt", function(data) {
		//angle = data
		//}, "text");
		var angle = globalData.random;

		context.setTransform(1,0,0,1,0,0); // reset transform.
 		context.translate(canvas.width/2, canvas.height/2);
		context.rotate(angle * Math.PI / 180);
		context.translate(-canvas.width/2, -canvas.height/2);

		context.clearRect(0, 0, canvas.width, canvas.height);
		context.drawImage(logoImage, 0,0);
	}
	setInterval(drawArrow, 250);
</script>



<hr>
Debug all: <div id="debug" style="">[LOADING]</div>


<?php dataRetrievalScript(); ?>



        </div><!-- /content -->
<?php printFooter(); ?>
    </div><!-- /page -->
</body>
</html>
<?php } ?>
