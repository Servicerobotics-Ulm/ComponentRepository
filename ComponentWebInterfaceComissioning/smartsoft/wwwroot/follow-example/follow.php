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
<body style='background-color: white' onLoad="updateModel()">
<style>
.ui-page {
background: white;
}
.personId {
text-align: left;
font-weight: bold;
font-size: 30pt;
margin: 5px;
float: left;
width: 50px;
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


<script src="./skripts/video_streaming.js"></script>

<script>

var person_id = 0;

function setPersonId(id) {
	person_id = id;
	updateModel();
}
function setPersonId() {
	person_id = document.getElementById("person_id").value;
	updateModel();
}

function updateModel() {
	var cmd="FOLLOWPERSON;";
	cmd += person_id + ";";
	var c = document.getElementById("follow_cmd").value=cmd;
}


</script>
<script src="./skripts/video_streaming.js"></script>

<div style="width:480px; height:270px;border-style: solid;border-width:1px">
	<image width='480' height='270' id='video_image' alt="Robot Video Stream" style="position:absolute; z-index:0; "></image>
	<div id="stream_indicator" style="position:relative; z-index:1; font-size: 30px; color:red; float:right; margin-top:49%;visibility:hidden;"> &#9679;</div>
</div>


<div style="width: 300px;">
	<div class="ui-block-b" style=""><button data-theme="b" onClick="startStopStream();" value="STARTSTREAM" id="start_stop_button">Start/Stop Stream</button></div> 
</div>


<div style="width: 300px;">
 <table style="width:100%;">
  <tr>
    <td>Follow Person ID:</td>
    <td><input class="personId" id="person_id" type="number" value="0" onClick="setPersonId();" onkeyup="setPersonId();" style="width: 80px;" ></td>
  </tr>
</table> 
</div>




<div style="clear:both;"></div>

<div class="ui-block-b">
<form action="<?=HOSTSCRIPT?>" method="get" target="_parent">
	<input id="prepare_follow_cmd" type="hidden" name="task" value="InitFollow">
	<button id="go" type="submit" data-transition="fade" data-ajax="false" data-theme="b">Detect Me</button>
</form>
</div>

<div class="ui-block-b">
<form action="<?=HOSTSCRIPT?>" method="get" target="_parent">
	<input id="follow_cmd" type="hidden" name="task" value="NO ORDER">
	<button id="go" type="submit" data-transition="fade" data-ajax="false" data-theme="b">Follow Me</button>
</form>
</div>


<div class="ui-block-b">
<form action="<?=HOSTSCRIPT?>" method="get" target="_parent">
	<input id="stop_cmd" type="hidden" name="task" value="STOPFOLLOWPERSON">
	<button id="go" type="submit" data-transition="fade" data-ajax="false" data-theme="b">Stop Follow</button>
</form>
</div>

<div class="ui-block-b">
<form action="<?=HOSTSCRIPT?>" method="get" target="_parent">
	<input id="deliver_cmd" type="hidden" name="task" value="Deliver">
	<button id="go" type="submit" data-transition="fade" data-ajax="false" data-theme="b">Deliver</button>
</form>
</div>













        </div><!-- /content -->


<?php printFooter(); ?>

    </div><!-- /page -->
</body>
</html>
<?php } ?>
