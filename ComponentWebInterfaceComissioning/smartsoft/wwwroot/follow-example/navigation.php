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
<body style='background-color: white'> 
<style>
.ui-page { 
background: white;
}
</style>
    <div data-role="page" id="info" >
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
        <div data-role="content" data-theme="d" style='overflow-y: hidden;'>    
<form action="<?=HOSTSCRIPT?>" method="get" target="_parent">
<fieldset class="ui-grid-a">
	<div class="ui-block-a">
		<select name="task" id="task" data-native-menu="false">
			<option value="WP1">Main Entrance</option>
			<option value="WP3">C26 - Service Robotics Lab</option>
			<option value="WP6">C13 - Computer Netw. Lab</option>
			<option value="WP8">C13 - Realtime Sys. Lab</option>
			<option value="WP7">C12 - CS Offices</option>
			<option value="WP0">C03 - Workshop</option>
		</select>
	</div>

	<div class="ui-block-b">
		<button type="submit" data-ajax="false" data-theme="b">Start</button>
	</div>	   
</fieldset>
</form> 






	<script type="text/javascript">
	 $(document).ready(function() {
	   $("#refresh").load("navigation-refresh.php");
	   var refresh = setInterval(function() {
	      $("#refresh").load('navigation-refresh.php');
	   }, 1000); //1000
	});
	</script>
<div id="refresh" style="text-align:center; height: 300px">loading..</div>


<div style='width: 99%; height: 400px; border: 2px black solid; overflow: auto; text-align: center;'>
<img id='map' src='map.jpg' style='' <?php $x = getimagesize("map.jpg"); echo $x[3]; ?> >
</div>

<script>
	var theimg = document.getElementById("map");
	setInterval(function() {
	    theimg.src = "map.jpg" + '?t=' + new Date().getTime();
	}, 1000);
</script>



        </div><!-- /content -->
<!--
        <div data-role="footer" style='position: absolute; bottom: 0px'>
            <h4>&nbsp;</h4>
        </div>
-->
    </div><!-- /page -->
</body>
</html>
<?php } ?>
