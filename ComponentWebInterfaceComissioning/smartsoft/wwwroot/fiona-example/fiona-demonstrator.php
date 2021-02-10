<?php
// --------------------------------------------------------------------------
//
//  Copyright (C) 2015 Dennis Stampfer
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
<body style='background-color: white'> 
<style>
.ui-page { 
background: white;
}
</style>
    <div data-role="page" id="info" >
	<?php 
	printHeader("Demonstrator");
	?>
        <nav data-role="navbar">
            <ul>
		<?php 
                printMenu();
		?>
            </ul>
        </nav>
        <div data-role="content" data-theme="d" style='overflow-y: hidden;'>   

<form action="<?=HOSTSCRIPT?>" method="get" target="_parent">
<fieldset class="ui-grid-a">
	<div class="ui-block-a">
		<select name="task" id="task" data-native-menu="false" onchange="disableButton()">
			<option selected="selected" disabled="true" />Choose Destination</option>

			<!-- HSU navigation -->
			<option value="PLAN;WP1;20.8903;-12.6157;0">Main Entrance</option>
			<option value="PLAN;WP3;-7.58988;3.08116;0">C26 - Service Robotics Lab</option>
			<option value="PLAN;WP6;-11.3284;18.0513;0">C13 - Computer Netw. Lab</option>
			<option value="PLAN;WP8;-11.3284;30.4237;0">C13 - Realtime Sys. Lab</option>
			<option value="PLAN;WP7;-11.3284;4.0644;0">C12 - CS Offices</option>
			<option value="PLAN;WP0;-35.0826;36.0742;0">C03 - Workshop</option>
			<option value="PLAN;WP14;2.62038;-17.0175;1">A112 - first floor</option>


			<!-- Comland -->
			<!--
			<option value="PLAN;WP36;-5.49981;-5.24121;0">Common Office</option>
			<option value="PLAN;WP4;-8.94484;2.00412;0">Small Meeting Room</option>
			<option value="PLAN;WP9;-4.06204;2.00412;0">Director's Office</option>
			<option value="PLAN;WP12;-0.613177;2.00412;0">Head Of Sales</option>
			<option value="PLAN;WP14;0.516422;2.00412;0">Server Room</option>
			<option value="PLAN;WP15;3.10334;2.9051;0">Storeroom</option>
			<option value="PLAN;WP17;5.22925;-0.730893;0">Archive</option>
			<option value="PLAN;WP19;6.43421;-4.07924;0">Cleaning Room</option>
			<option value="PLAN;WP54;13.9726;0.538164;0">Showroom</option>
			<option value="PLAN;WP47;10.0715;0.538164;0">Dining Room</option>
			<option value="PLAN;WP58;11.7994;2.66415;0">Large Meeting Room</option>
			<option value="PLAN;WP53;8.68156;-1.98011;0">Kitchen</option>
			<option value="PLAN;WP51;10.0715;-0.4377317;0">WC-f</option>
			<option value="PLAN;WP21;10.6251;-3.84532;0">WC-m</option>
			<option value="PLAN;WP22;-12.3936;1.48519;0">Microsoft Department</option>
			-->
		</select>
	</div>

	<div class="ui-block-b">
		<button id="startButton" type="submit" data-ajax="false" data-theme="b" disabled="true">Start</button>
	</div>	   
</fieldset>
</form>
<script>
	function disableButton() {
		var select = document.getElementById("task");
		var op = select.options[select.selectedIndex];
		if(op.disabled == true){
			document.getElementById("startButton").disabled = true;
		}else{
			document.getElementById("startButton").disabled = false;
		}
		$('[type="submit"]').button('refresh');
	}
</script>

<form action="<?=HOSTSCRIPT?>" method="get" target="_parent">
<fieldset class="ui-grid-a">
	<div class="ui-block-a" style="visibility: hidden"> 
		<select name="task" id="task" data-native-menu="false">
			<option value="STOP">Stop</option>
		</select>
	</div>
	<div class="ui-block-b">
		<button type="submit" data-ajax="false" data-theme="b" >Stop</button>
	</div>	   
</fieldset>
</form>

System Status: <span id="status">[LOADING]</span>
<?php dataRetrievalScript(250); ?>

<div style="background-color:#F2F2F2; padding:10px">

<table style="width: 100%">
<tr><td style="text-align: left; vertical-align: top; width:20%">

<td style="text-align: center; vertical-align: top; width: 60%">


Next waypoint direction:<br>
<!-- ----------------------------- -->
<!--      CANVAS                   -->
<!-- ----------------------------- -->
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
		var angle = globalData.nextWaypoint;

		context.setTransform(1,0,0,1,0,0); // reset transform.
 		context.translate(canvas.width/2, canvas.height/2);
		context.rotate(angle * Math.PI / 180);
		context.translate(-canvas.width/2, -canvas.height/2);

		context.clearRect(0, 0, canvas.width, canvas.height);
		context.drawImage(logoImage, 0,0);
	}
	setInterval(drawArrow, 250);
</script>
<br>
<span id="nextWaypoint"></span>&deg;

<td style="text-align: right; vertical-align: bottom; width: 20%">

<label for="autoscroll">autoscroll:</label>
<select name="autoscroll" id="autoscroll" data-role="slider" >        
	<option value="off">Off</option>
	<option value="on" selected>On</option>
</select>

<!--Next waypoint:<br>
<span id="nextWaypoint"></span> -->


</tr></table>


<div id="scroll" style='width: 99%; height: 400px; border: 2px black solid; overflow: auto; text-align: center;'> 
	<canvas id="map" width="736" height="806"></canvas>
</div>
<script> 
function drawMap() {
	var c = document.getElementById("map");
	var ctx = c.getContext("2d");
	ctx.clearRect(0, 0, map.width, map.height);

	//map
	var mapImage = new Image();
	var fileName = globalData.map;
	mapImage.src = fileName;
	c.height = mapImage.height;
	c.width = mapImage.width;
	ctx.drawImage(mapImage,0,0);
	

	//path
	ctx.lineWidth=5;
	ctx.strokeStyle="#104e8b";
	var path = globalData.path;
	var location_z = globalData.location_z;
	var x = globalData.location_px[0];
	var y = globalData.location_px[1];

	ctx.beginPath();	//same floor
	ctx.moveTo(x,y);
	for (var i = 0; i < path.length; ++i) {	
		if(path[i][2] != location_z){
			ctx.moveTo(path[i][0],path[i][1]);
		}else{
			ctx.lineTo(path[i][0],path[i][1]);
		}		
	}
	ctx.stroke();
	ctx.closePath();

	ctx.save();	//other floor -> dashed line
	if(ctx.setLineDash){
		ctx.setLineDash([5]);
	}
	ctx.beginPath();	
	ctx.moveTo(x,y);
	for (var i = 0; i < path.length; ++i) {
		if(path[i][2] == location_z){
			ctx.moveTo(path[i][0],path[i][1]);
		}else{
			ctx.lineTo(path[i][0],path[i][1]);
		}
	}
	ctx.stroke();
	ctx.closePath();
	ctx.restore();


	//destination
	ctx.beginPath();
	ctx.fillStyle="#104e8b";
	if(path.length > 0){
		ctx.arc(path[path.length-1][0],path[path.length-1][1],7,0,2*Math.PI);
		if(path[path.length-1][2] == location_z){
			ctx.fill()
		}
	}
	ctx.stroke();
	ctx.closePath();


	//location
	var locationImage = new Image(); 
	locationImage.src = 'positionIcon.png';
	var orientation = globalData.orientation;
 	ctx.save();
	ctx.translate(x, y);
  	ctx.rotate(-orientation*Math.PI / 180);
  	ctx.translate(-x, -y);
  	ctx.drawImage(locationImage, x-12, y-12);  
  	ctx.restore();


	//autoscroll
	if($('#autoscroll').val() == "on"){
		var objDiv = document.getElementById("scroll");
		objDiv.scrollTop = y - (objDiv.offsetHeight/2);
		objDiv.scrollLeft = x - (objDiv.offsetWidth/2);
	}


}setInterval(drawMap, 250);

</script>


<!-- OLD MAP IMPLEMENTATION (REFRESHES MAP IMAGE GENERATED BY COMPONENT)
<div style='width: 99%; height: 400px; border: 2px black solid; overflow: auto; text-align: center;'> 
<img id='map' src='map.jpg' style='' <?php $x = getimagesize("map.jpg"); echo $x[3]; ?> >
</div>

<script>
	var theimg = document.getElementById("map");
	setInterval(function() {
	    theimg.src = "map.jpg" + '?t=' + new Date().getTime();
	}, 1000);
</script>
-->

</div>

        </div><!-- /content -->


<?php printFooter(); ?>

    </div><!-- /page -->
</body>
</html>
<?php } ?>
