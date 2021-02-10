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
.qty {
text-align: center;
font-weight: bold;
font-size: 30pt;
margin: 0px;
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


<script>

var goods = [
	"SMACKS", 
	"CORN-FLAKES",
	"FROSTIES",
	"SPECIAL",
	"CHOCO"
];

var qty = [0, 0, 0, 0, 0];


function addGood(g) {
	var idx=goods.indexOf(g);
	qty[idx] = qty[idx]+1;
	updateModel();
}
function removeGood(g) {
	var idx=goods.indexOf(g);
	if(qty[idx] > 0) {
		qty[idx] = qty[idx]-1;
	}
	updateModel();
}

function updateModel() {
	var cmd="ORDER;";
	for (index = 0; index < qty.length; ++index) {
	    cmd += goods[index] + ";" + qty[index] + ";";
            document.getElementById(goods[index]+"_qty").innerHTML=qty[index];
	}
	var c = document.getElementById("cmd").value=cmd;

}


</script> 

<div class="ui-block-b" style="margin: 10px; text-align: center;">
<img src="SMACKS.jpg">
<p class="qty" id="SMACKS_qty">?</p>
<button data-theme="b" onClick="addGood('SMACKS');">+</button>
<button data-theme="b" onClick="removeGood('SMACKS');">-</button>
</div>

<div class="ui-block-b" style="margin: 10px; text-align: center;">
<img src="CORN-FLAKES.jpg">
<p class="qty"  id="CORN-FLAKES_qty">?</p>
<button data-theme="b" onClick="addGood('CORN-FLAKES');">+</button>
<button data-theme="b" onClick="removeGood('CORN-FLAKES');">-</button>
</div>

<div class="ui-block-b" style="margin: 10px; text-align: center;">
<img src="FROSTIES.jpg">
<p class="qty"  id="FROSTIES_qty">?</p>
<button data-theme="b" onClick="addGood('FROSTIES');">+</button>
<button data-theme="b" onClick="removeGood('FROSTIES');">-</button>
</div>

<div class="ui-block-b" style="margin: 10px; text-align: center;">
<img src="SPECIAL.jpg">
<p class="qty"  id="SPECIAL_qty">?</p>
<button data-theme="b" onClick="addGood('SPECIAL');">+</button>
<button data-theme="b" onClick="removeGood('SPECIAL');">-</button>
</div>

<div class="ui-block-b" style="margin: 10px; text-align: center;">
<img src="CHOCO.jpg">
<p class="qty"  id="CHOCO_qty">?</p>
<button data-theme="b" onClick="addGood('CHOCO');">+</button>
<button data-theme="b" onClick="removeGood('CHOCO');">-</button>
</div>


<div style="clear:both;"></div>

<div class="ui-block-b">
<form action="<?=HOSTSCRIPT?>" method="get" target="_parent">
	<input id="cmd" type="hidden" name="task" value="NO ORDER">
	<button id="go" type="submit" data-transition="fade" data-ajax="false" data-theme="b">Queue Order</button>
</form>
</div>














        </div><!-- /content -->


<?php printFooter(); ?>

    </div><!-- /page -->
</body>
</html>
<?php } ?>
